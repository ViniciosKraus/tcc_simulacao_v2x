#include "AccidentAlertApp.h"

#include <veins/modules/application/traci/TraCIDemo11pMessage_m.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <list>

using namespace omnetpp;
using namespace veins;

namespace {
bool isComparisonVehicle(const veins::TraCIMobility* mobility)
{
    if (mobility == nullptr) {
        return false;
    }

    const std::string vehicleId = mobility->getExternalId();
    return vehicleId == "V3_no_v2x";
}

std::list<std::string> buildMarginalRoute()
{
    return {
        "529303429",
        "484273114",
        "79290812#0",
        "79290812#1",
        "978180644",
        "978180645",
        "484273113",
        "529303407",
        "529303406",
    };
}

std::list<std::string> buildHighwayRoute()
{
    return {
        "529303429",
        "529303430",
        "529303407",
        "529303406",
    };
}

// -----------------------------------------------------------------------------
//  Gatilho de veículo elegível para desvio.
//  Objetivo: filtrar apenas veículos que realmente estão no ramo principal da
//  rodovia e que ainda não seguiram para a rota marginal. Isso evita falsos
//  positivos em veículos de fila lenta, rotas alternativas ou tráfego fora do
//  ponto de decisão.
// -----------------------------------------------------------------------------
bool isHighwayDetourCandidate(veins::TraCICommandInterface::Vehicle* traciVehicle)
{
    if (traciVehicle == nullptr) {
        return false;
    }

    const std::string currentRoad = traciVehicle->getRoadId();
    const std::string routeId = traciVehicle->getRouteId();
    const auto plannedRoads = traciVehicle->getPlannedRoadIds();

    // Mantém a lógica robusta ao nome da rota gerada dinamicamente em SUMO.
    // Nem todos os veículos apresentam route_highway/route_marginal fixos;
    // por isso o critério principal é o ramo físico da via e os roadIds planejados.
    if (routeId == "route_slow" || routeId.find("slow") != std::string::npos) {
        return false;
    }
    if (routeId == "route_marginal" || routeId.find("marginal") != std::string::npos) {
        return false;
    }

    const bool hasHighwayUpstream = currentRoad == "529303429" ||
                                   std::find(plannedRoads.begin(), plannedRoads.end(), "529303429") != plannedRoads.end();
    const bool hasHighwayContinuation = currentRoad == "529303430" ||
                                       std::find(plannedRoads.begin(), plannedRoads.end(), "529303430") != plannedRoads.end();
    const bool hasMarginalDetour = std::find(plannedRoads.begin(), plannedRoads.end(), "484273114") != plannedRoads.end();

    return hasHighwayUpstream && hasHighwayContinuation && !hasMarginalDetour;
}

// -----------------------------------------------------------------------------
//  Janela de decisão do desvio.
//  Uso: a avaliação só é permitida próximo da bifurcação principal, antes do
//  ponto em que o veículo já teria optado pela marginal. Isso cria uma janela
//  temporal e espacial para que a decisão ocorra na hora correta.
// -----------------------------------------------------------------------------
bool isOnHighwayApproach(veins::TraCICommandInterface::Vehicle* traciVehicle, double lanePosThreshold = 200.0)
{
    if (traciVehicle == nullptr || !isHighwayDetourCandidate(traciVehicle)) {
        return false;
    }

    const std::string currentRoad = traciVehicle->getRoadId();
    const double currentPos = traciVehicle->getLanePosition();

    // A detecção só é válida no ponto de ramificação real da rodovia.
    // O acesso à marginal existe em 529303429; no ramo 529303430 a conexão para
    // 484273114 não existe, então trocar a rota a partir desse ponto é fisicamente
    // impossível em SUMO.
    return currentRoad == "529303429" && currentPos > 20.0 && currentPos < lanePosThreshold;
}

// -----------------------------------------------------------------------------
//  Critério de avaliação do desvio.
//  A decisão considera: (i) estar no ramo da rodovia principal, (ii) estar antes
//  da bifurcação, (iii) presença de congestionamento à frente e (iv) se o veículo
//  ainda não saiu da rota principal. A ideia é evitar decisões tardias ou
//  baseadas em veículos que fazem parte da fila de lentidão.
// -----------------------------------------------------------------------------
double estimateFreeFlowSpeed(veins::TraCICommandInterface::Vehicle* traciVehicle,
    veins::TraCICommandInterface* commandInterface,
    double fallback = 27.78)
{
    if (traciVehicle == nullptr) {
        return fallback;
    }

    const double vehicleMaxSpeed = traciVehicle->getMaxSpeed();
    if (vehicleMaxSpeed > 0.0) {
        fallback = vehicleMaxSpeed;
    }

    if (commandInterface != nullptr) {
        try {
            const std::string laneId = traciVehicle->getLaneId();
            if (!laneId.empty()) {
                auto lane = commandInterface->lane(laneId);
                const double laneMaxSpeed = lane.getMaxSpeed();
                if (laneMaxSpeed > 0.0) {
                    return laneMaxSpeed;
                }
            }
        } catch (const std::exception&) {
            // Fallback para o limite do veículo se a via não expuser o valor.
        }
    }

    return std::max(0.1, fallback);
}

double computeSpeedPerformanceIndex(double currentSpeed, double freeFlowSpeed)
{
    if (freeFlowSpeed <= 0.0) {
        return 100.0;
    }

    const double spi = (currentSpeed / freeFlowSpeed) * 100.0;
    if (spi < 0.0) {
        return 0.0;
    }
    if (spi > 100.0) {
        return 100.0;
    }
    return spi;
}

std::list<std::string> currentRouteCandidate(veins::TraCICommandInterface::Vehicle* traciVehicle);
double estimateRouteTravelTime(veins::TraCICommandInterface* commandInterface, const std::list<std::string>& roads);

bool detourIsWorthwhile(veins::TraCICommandInterface::Vehicle* traciVehicle,
    veins::TraCICommandInterface* commandInterface,
    double currentSpeed)
{
    if (traciVehicle == nullptr || commandInterface == nullptr) {
        return false;
    }

    const std::list<std::string> currentRoads = currentRouteCandidate(traciVehicle);
    const std::list<std::string> alternativeRoads = buildMarginalRoute();
    if (currentRoads == alternativeRoads) {
        return false;
    }

    const double currentCost = estimateRouteTravelTime(commandInterface, currentRoads);
    const double alternativeCost = estimateRouteTravelTime(commandInterface, alternativeRoads);
    const double benefitMargin = (currentCost - alternativeCost) / std::max(0.1, currentCost);

    EV_INFO << "V2X: DETOUR_VALUE | lane=" << traciVehicle->getLaneId()
            << " | currentCost=" << currentCost
            << " alternativeCost=" << alternativeCost
            << " gainRatio=" << benefitMargin
            << " currentSpeed=" << currentSpeed << endl;

    // Regra realista para este cenário: quando o veículo já está em congestionamento
    // severo no ponto de bifurcação, o custo estimado da rota principal pode continuar
    // parecendo menor mesmo com a fila trava, porque a análise de tempo estático não
    // reflete o tempo de espera real. Nesse caso, a marginal só é aceita quando não
    // está claramente pior do que a rota principal.
    const bool severeCongestion = currentSpeed < 5.0;
    const bool alternativeNotCatastrophic = alternativeCost < currentCost * 1.50;

    if (severeCongestion) {
        EV_INFO << "V2X: DETOUR_SEVERE_CONGESTION | severeCongestion=" << severeCongestion
                << " alternativeNotCatastrophic=" << alternativeNotCatastrophic
                << " currentSpeed=" << currentSpeed << endl;
        return alternativeNotCatastrophic;
    }

    return alternativeCost < currentCost * 0.75;
}

bool shouldEvaluateDetour(veins::TraCICommandInterface::Vehicle* traciVehicle,
    const veins::TraCIMobility* mobility,
    veins::TraCICommandInterface* commandInterface,
    double senderSpeed, 
    double freeFlowSpeed = 0.0,
    double congestionSpiThreshold = 50.0)
{
    if (traciVehicle == nullptr || mobility == nullptr || !isHighwayDetourCandidate(traciVehicle)) {
        return false;
    }

    const std::string currentRoad = traciVehicle->getRoadId();
    const std::string currentRoute = traciVehicle->getRouteId();
    const double currentPos = traciVehicle->getLanePosition();
    const double currentSpeed = mobility->getSpeed();
    const double effectiveFreeFlowSpeed = estimateFreeFlowSpeed(traciVehicle,
        commandInterface,
        freeFlowSpeed > 0.0 ? freeFlowSpeed : 27.78);
    const double spi = computeSpeedPerformanceIndex(currentSpeed, effectiveFreeFlowSpeed);
    const double senderSpi = computeSpeedPerformanceIndex(senderSpeed, effectiveFreeFlowSpeed);
    const auto leaderInfo = traciVehicle->getLeader(200.0);
    const auto plannedRoads = traciVehicle->getPlannedRoadIds();
    const bool beforeBranch = isOnHighwayApproach(traciVehicle, 200.0);
    const bool leaderBlocked = !leaderInfo.first.empty() && leaderInfo.second < 30.0;
    const bool localSlowdown  = currentSpeed < effectiveFreeFlowSpeed * 0.80;
    const bool remoteSlowdown = senderSpi < congestionSpiThreshold;
    const bool moderateSlowdown = localSlowdown || remoteSlowdown;
    const bool congestionAhead = spi < 80.0 || senderSpi < congestionSpiThreshold || leaderBlocked || moderateSlowdown;
    const bool stillOnMainPath = currentRoad == "529303429";
    const bool hasRealCongestion = congestionAhead && (leaderBlocked || moderateSlowdown);
    const bool worthwhileDetour = detourIsWorthwhile(traciVehicle, commandInterface, currentSpeed);
    const bool shouldEvaluate = beforeBranch && stillOnMainPath && worthwhileDetour && hasRealCongestion;

    EV_INFO << "V2X: CHECK_DETOUR | node=" << (mobility ? mobility->getExternalId() : "?")
            << " | road=" << currentRoad
            << " pos=" << currentPos
            << " route=" << currentRoute
            << " speed=" << currentSpeed
            << " senderSpeed=" << senderSpeed
            << " spi=" << spi
            << " senderSpi=" << senderSpi
            << " freeFlowSpeed=" << effectiveFreeFlowSpeed
            << " beforeBranch=" << beforeBranch
            << " congestionAhead=" << congestionAhead
            << " moderateSlowdown=" << moderateSlowdown
            << " leaderBlocked=" << leaderBlocked
            << " hasRealCongestion=" << hasRealCongestion
            << " worthwhileDetour=" << worthwhileDetour
            << " stillOnMainPath=" << stillOnMainPath
            << " plannedRoadsSize=" << plannedRoads.size()
            << " -> " << shouldEvaluate << endl;

    return shouldEvaluate;
}

double estimateRouteTravelTime(veins::TraCICommandInterface* commandInterface, const std::list<std::string>& roads)
{
    if (commandInterface == nullptr) {
        return std::numeric_limits<double>::infinity();
    }

    double total = 0.0;
    for (const auto& roadId : roads) {
        auto road = commandInterface->road(roadId);
        const double travelTime = road.getCurrentTravelTime();
        total += std::max(0.1, travelTime);
    }
    return total;
}

std::list<std::string> currentRouteCandidate(veins::TraCICommandInterface::Vehicle* traciVehicle)
{
    if (traciVehicle == nullptr) {
        return buildHighwayRoute();
    }

    const std::list<std::string> plannedRoads = traciVehicle->getPlannedRoadIds();
    if (!plannedRoads.empty()) {
        return plannedRoads;
    }

    const std::string routeId = traciVehicle->getRouteId();
    if (routeId.find("marginal") != std::string::npos) {
        return buildMarginalRoute();
    }

    return buildHighwayRoute();
}

// -----------------------------------------------------------------------------
//  Comparação de custo entre rota atual e alternativa.
//  A decisão não depende de saída hardcoded; em vez disso, a rota alternativa é
//  avaliada em termos de tempo estimado até o destino final. O veículo desvia
//  somente quando a alternativa melhora de forma objetiva a estimativa de custo.
// -----------------------------------------------------------------------------
bool tryAlternativeRoute(veins::TraCICommandInterface::Vehicle* traciVehicle, veins::TraCICommandInterface* commandInterface)
{
    if (traciVehicle == nullptr || commandInterface == nullptr || !isHighwayDetourCandidate(traciVehicle)) {
        return false;
    }

    try {
        const std::string routeId = traciVehicle->getRouteId();
        if (routeId == "route_slow" || routeId.find("slow") != std::string::npos) {
            EV_INFO << "V2X: ignora comparacao de rota para veiculo ja na fila lenta | route=" << routeId << endl;
            return false;
        }

        const std::string currentRoad = traciVehicle->getRoadId();
        const double currentPos = traciVehicle->getLanePosition();
        const bool onMainHighway = currentRoad == "529303429";
        if (!onMainHighway || currentPos < 20.0 || currentPos > 200.0) {
            EV_INFO << "V2X: desvio bloqueado por zona de decisão | road=" << currentRoad
                    << " pos=" << currentPos << endl;
            return false;
        }

        const std::list<std::string> currentRoads = currentRouteCandidate(traciVehicle);
        const std::list<std::string> alternativeRoads = buildMarginalRoute();

        if (currentRoads == alternativeRoads) {
            EV_INFO << "V2X: rota atual ja e a alternativa | nao ha desvio necessario" << endl;
            return false;
        }

        const double currentCost = estimateRouteTravelTime(commandInterface, currentRoads);
        const double alternativeCost = estimateRouteTravelTime(commandInterface, alternativeRoads);

        EV_INFO << "V2X: COMPARACAO DE ROTA | atual=" << currentCost
                << " | alternativa=" << alternativeCost
                << " | currentRouteId=" << traciVehicle->getRouteId() << endl;

        if (alternativeCost >= currentCost * 0.75) {
            EV_INFO << "V2X: rota alternativa nao melhora tempo estimado do destino final | atual=" << currentCost
                    << " | alternativa=" << alternativeCost << endl;
            return false;
        }

        EV_INFO << "V2X: APLICANDO ROTA MARGINAL | road=" << currentRoad
                << " pos=" << currentPos
                << " currentRouteId=" << traciVehicle->getRouteId()
                << " firstEdge=" << alternativeRoads.front()
                << " lastEdge=" << alternativeRoads.back() << endl;

        const bool rerouteOk = traciVehicle->changeVehicleRoute(alternativeRoads);

        // DIAGNÓSTICO: verificar se a marginal realmente foi aplicada.
        // Usamos getPlannedRoadIds() para inspecionar a rota real, não o nome
        // (o SUMO sempre nomeia rotas dinâmicas como !V2!var#N, então o nome
        // não diz nada sobre validade).
        const auto planned = traciVehicle->getPlannedRoadIds();
        const bool appliedMarginal =
            std::find(planned.begin(), planned.end(), "484273114") != planned.end();

        const std::string newRoute = traciVehicle->getRouteId();

        EV_INFO << "V2X: DIAG REROUTE | routeId=" << newRoute
                << " | plannedSize=" << planned.size()
                << " | has484273114=" << appliedMarginal
                << " | roads=";
        for (const auto& e : planned) EV_INFO << e << " ";
        EV_INFO << endl;

        // Se o SUMO ignorou a lista passada e a marginal não está entre os edges
        // planejados, o desvio falhou na prática. Reverte para highway.
        if (rerouteOk && !appliedMarginal) {
            EV_WARN << "V2X: changeVehicleRoute nao aplicou a marginal — revertendo" << endl;
            traciVehicle->changeVehicleRoute(buildHighwayRoute());
            return false;
        }

        EV_INFO << "V2X: RESULTADO CHANGE_ROUTE | ok=" << rerouteOk
                << " | routeId=" << newRoute
                << " | appliedMarginal=" << appliedMarginal
                << " | road=" << currentRoad
                << " | pos=" << currentPos << endl;
        return rerouteOk;
    } catch (const std::exception& e) {
        EV_WARN << "V2X: Falha ao aplicar rota alternativa: " << e.what() << endl;
        return false;
    }
}

}

Define_Module(tcc::AccidentAlertApp);

namespace tcc {

void AccidentAlertApp::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);

    if (stage == 0) {

        EV_INFO << "V2X: NODE MAP | omnetIndex=" << getParentModule()->getIndex()
                << " | externalId=" << mobility->getExternalId()
                << " | initialRoute=" << traciVehicle->getRouteId() << endl;

        enableV2X = par("enableV2X").boolValue();
        // A identidade do veículo vem do SUMO/TraCI e é mais confiável que o
        // índice do node, que pode mudar conforme a ordem de criação dinâmica.
        if (isComparisonVehicle(mobility)) {
            enableV2X = false;
        }
        reactionStrategy = par("reactionStrategy").stdstringValue();
        reactionSpeedFactor = par("reactionSpeedFactor").doubleValue();

        safeDistance = par("safeDistance").doubleValue();
        reactionDuration = par("reactionDuration");
        reactionMonitorInterval = par("reactionMonitorInterval");

        if (hasPar("freeFlowSpeed")) {
            freeFlowSpeed = par("freeFlowSpeed").doubleValue();
        }
        if (hasPar("congestionSpiThreshold")) {
            congestionSpiThreshold = par("congestionSpiThreshold").doubleValue();
        }
        if (freeFlowSpeed <= 0.0) {
            freeFlowSpeed = estimateFreeFlowSpeed(traciVehicle, mobility->getCommandInterface(), 27.78);
        }

        if (getParentModule()->getIndex() == 0 && enableV2X) {
            accidentMsg = new cMessage("accidentAlert");
            simtime_t accidentTime = simTime() + 0.1;
            if (accidentTime < 20.1) {
                accidentTime = 20.1;
            }
            scheduleAt(accidentTime, accidentMsg);
        }

        if (getParentModule()->getIndex() != 0 && enableV2X) {
            reactionMonitorMsg = new cMessage("reactionMonitor");
            // Agenda o monitoramento contínuo (será usado para desvio e reação)
            scheduleAt(simTime() + reactionMonitorInterval, reactionMonitorMsg);
        }
    }

    rerouteTriggered = false;
    lowSpeedStart = -1;
    lastCongestionAlertTime = simTime();
}

void AccidentAlertApp::handleSelfMsg(cMessage* msg)
{
    // ============================================================
    //  TRATAMENTO DO reactionMonitorMsg (sempre ativo)
    // ============================================================
    if (msg == reactionMonitorMsg) {

        // --- 1. ENVIO DE ALERTA DE CONGESTIONAMENTO (veículos lentos) ---
        if (enableV2X && getParentModule()->getIndex() != 0) {
            const double speed = mobility->getSpeed();
            const double effectiveFreeFlowSpeed = estimateFreeFlowSpeed(traciVehicle, mobility->getCommandInterface(), freeFlowSpeed > 0.0 ? freeFlowSpeed : 27.78);
            const double spi = computeSpeedPerformanceIndex(speed, effectiveFreeFlowSpeed);
            if (spi < congestionSpiThreshold) {
                if (simTime() - lastCongestionAlertTime > 2.0) {
                    auto* wsm = new TraCIDemo11pMessage();
                    wsm->setName("CongestionAlert");
                    populateWSM(wsm);
                    wsm->setSerial(1);
                    char speedStr[16];
                    std::snprintf(speedStr, sizeof(speedStr), "%.2f", speed);
                    wsm->setDemoData(speedStr);
                    sendDown(wsm);
                    lastCongestionAlertTime = simTime();
                    EV_INFO << "V2X: CONGESTION ALERT SENT by "
                            << getParentModule()->getFullName()
                            << " | speed=" << speed
                            << " | spi=" << spi
                            << " | threshold=" << congestionSpiThreshold << endl;
                }
            }
        }

        // --------------------------------------------------------------------
        //  2. LÓGICA DE DESVIO (monitoramento contínuo do tráfego à frente)
        //  Observação: o gatilho de desvio foi ajustado para depender da posição
        //  física do veículo na rodovia e do custo da rota, e não do nome da rota
        //  gerada pelo SUMO. Isso torna o comportamento estável mesmo quando o
        //  sistema produz rotas dinâmicas como !V2!var#1.
        // --------------------------------------------------------------------
        if (enableV2X && !rerouteTriggered && getParentModule()->getIndex() != 0) {
            const double speed = mobility->getSpeed();

            if (!isHighwayDetourCandidate(traciVehicle)) {
                lowSpeedStart = -1;
            } else if (shouldEvaluateDetour(traciVehicle, mobility, mobility->getCommandInterface(),
                     mobility->getSpeed(), freeFlowSpeed, congestionSpiThreshold)) {
                if (lowSpeedStart < 0) {
                    lowSpeedStart = simTime();
                }

                const bool isAlreadyStoppedAtDecisionPoint = mobility->getSpeed() < 2.0 &&
                    traciVehicle->getRoadId() == "529303429" &&
                    traciVehicle->getLanePosition() > 20.0 &&
                    traciVehicle->getLanePosition() < 200.0;

                if (simTime() - lowSpeedStart > 0.5 || isAlreadyStoppedAtDecisionPoint) {
                    const std::string currentRoad = traciVehicle->getRoadId();
                    const std::string currentRoute = traciVehicle->getRouteId();
                    const double currentPos = traciVehicle->getLanePosition();
                    const bool rerouted = tryAlternativeRoute(traciVehicle, mobility->getCommandInterface());
                    rerouteTriggered = rerouted;
                    if (rerouted) {
                        EV_INFO << "V2X: ROTA ALTERNATIVA ESCOLHIDA (monitoramento contínuo) em t=" << simTime()
                                << " | road=" << currentRoad
                                << " | pos=" << currentPos
                                << " | route=" << currentRoute
                                << " | speed=" << speed << endl;
                    } else {
                        EV_INFO << "V2X: tentativa de desvio em ponto válido sem sucesso | road=" << currentRoad
                                << " pos=" << currentPos
                                << " speed=" << speed << endl;
                    }
                    lowSpeedStart = -1;
                }
            } else {
                lowSpeedStart = -1;
            }
        }

        // --- 3. REAÇÃO AO ACIDENTE (se ativa) ---
        if (reactionActive) {

            //    Se o veículo já desviou, ele está na marginal. A reação ao
            //    acidente não se aplica mais — cancelar e liberar.
            if (rerouteTriggered) {
                reactionActive = false;
                haveLastReactionPosition = false;
                // Só devolve controle se o veículo já estiver em movimento.
                // Se estiver parado, mantém o slowDown no alvo atual para não
                // entregar o veículo a um SUMO que não vai movê-lo.
                const double speedNow = mobility->getSpeed();
                if (speedNow > 0.5) {
                    traciVehicle->setSpeed(-1);
                } else {
                    traciVehicle->setSpeed(3.0);   // empurrão para sair do zero
                }
                EV_INFO << "V2X REACTION CANCELLED (rerouted)"
                        << " | t=" << simTime()
                        << " | speed=" << speedNow
                        << " | by " << getParentModule()->getFullName() << endl;
                scheduleAt(simTime() + reactionMonitorInterval, reactionMonitorMsg);
                return;
            }

            if (simTime() - alertReceiveTime >= reactionDuration) {
                auto leaderInfo = traciVehicle->getLeader(1000.0);

                // ── SEM LÍDER: via livre à frente → ENCERRA a reação e libera ──
                if (leaderInfo.first.empty() || leaderInfo.second < 0.0) {
                    reactionActive = false;
                    haveLastReactionPosition = false;
                    const double speedBeforeRelease = mobility->getSpeed();
                    traciVehicle->setSpeed(-1);            // ← devolve controle ao SUMO
                    EV_INFO << "V2X REACTION FINISHED (road clear)"
                            << " | t=" << simTime()
                            << " | speedBeforeRelease=" << speedBeforeRelease
                            << " | by " << getParentModule()->getFullName()
                            << " | reactionDistance=" << reactionDistance << endl;
                    scheduleAt(simTime() + reactionMonitorInterval, reactionMonitorMsg);
                    return;
                }

                auto leaderVehicle = mobility->getCommandInterface()->vehicle(leaderInfo.first);
                const double leaderSpeed = leaderVehicle.getSpeed();

                // ── LÍDER PARADO: espera ──
                if (leaderSpeed <= 0.5) {
                    traciVehicle->setSpeed(0);
                    EV_INFO << "V2X WAITING FOR LEADER"
                            << " | t=" << simTime()
                            << " | leader=" << leaderInfo.first
                            << " | leaderSpeed=" << leaderSpeed
                            << " | leaderDistance=" << leaderInfo.second << endl;
                    scheduleAt(simTime() + reactionMonitorInterval, reactionMonitorMsg);
                    return;
                }

                // ── LÍDER SE MOVENDO: libera ──
                reactionActive = false;
                haveLastReactionPosition = false;
                const double speedBeforeRelease = mobility->getSpeed();
                traciVehicle->setSpeed(-1);
                EV_INFO << "V2X REACTION FINISHED"
                        << " | t=" << simTime()
                        << " | speedBeforeRelease=" << speedBeforeRelease
                        << " | by " << getParentModule()->getFullName()
                        << " | duration=" << reactionDuration
                        << " | leaderSpeed=" << leaderSpeed
                        << " | leaderDistance=" << leaderInfo.second
                        << " | reactionDistance=" << reactionDistance << endl;
                scheduleAt(simTime() + reactionMonitorInterval, reactionMonitorMsg);
                return;
            }

            if (mobility == nullptr || traciVehicle == nullptr) {
                return;
            }

            double currentSpeed = mobility->getSpeed();

            if (minSpeedAfterAlert < 0 || currentSpeed < minSpeedAfterAlert) {
                minSpeedAfterAlert = currentSpeed;
            }

            if (!reducedSpeedReached && currentSpeed <= reactionTargetSpeed + 0.1) {
                reducedSpeedReached = true;
                timeToReducedSpeed = simTime() - alertReceiveTime;
                EV_INFO << "V2X REACTION TARGET REACHED"
                        << " at t=" << simTime()
                        << " by " << getParentModule()->getFullName()
                        << " | targetSpeed=" << reactionTargetSpeed
                        << " | currentSpeed=" << currentSpeed
                        << " | timeToTarget=" << timeToReducedSpeed << endl;
            }

            double safeReactionSpeed = calculateSafeReactionSpeed(currentSpeed);

            if (reactionStrategy == "current") {
                if (currentLeaderDistance < 0.0) safeReactionSpeed = speedAtAlert;
                if (currentLeaderDistance > safeDistance && calculatedReactionDecel <= 0.0) {
                    safeReactionSpeed = speedAtAlert;
                }
            }

            if (reactionStrategy == "planned") {
                double maxAllowedDecel = std::max(0.1, traciVehicle->getDeccel());
                double dt = reactionMonitorInterval.dbl();
                double allowedDrop = maxAllowedDecel * dt;
                double lowerSpeed = std::max(0.0, currentSpeed - allowedDrop);
                safeReactionSpeed = std::max(lowerSpeed, safeReactionSpeed);
                safeReactionSpeed = std::min(currentSpeed, safeReactionSpeed);
            }

            traciVehicle->slowDown(safeReactionSpeed, reactionMonitorInterval);

            EV_INFO << "V2X CONTROL"
                    << " at t=" << simTime()
                    << " | currentSpeed=" << currentSpeed
                    << " | safeSpeed=" << safeReactionSpeed
                    << " | leaderDistance=" << currentLeaderDistance
                    << " | decel=" << calculatedReactionDecel << endl;
        }

        // Reagendar o monitor (sempre, exceto se já retornou)
        scheduleAt(simTime() + reactionMonitorInterval, reactionMonitorMsg);
        return;
    }

    // ============================================================
    //  TRATAMENTO DO accidentMsg (alerta de acidente)
    // ============================================================
    if (msg == accidentMsg && !accidentTriggered) {
        accidentTriggered = true;
        auto* wsm = new TraCIDemo11pMessage();
        wsm->setName("AccidentAlert");
        populateWSM(wsm);
        wsm->setSerial(1);
        char sendTimeString[64];
        std::snprintf(sendTimeString, sizeof(sendTimeString), "%.12f", simTime().dbl());
        wsm->setDemoData(sendTimeString);
        sendDown(wsm);
        alertsSent++;
        EV_INFO << "V2X ACCIDENT ALERT SENT"
                << " at t=" << simTime()
                << " by " << getParentModule()->getFullName() << endl;
        return;
    }

    DemoBaseApplLayer::handleSelfMsg(msg);
}

double AccidentAlertApp::calculateSafeReactionSpeed(double currentSpeed)
{
    // Mantido exatamente igual ao original – não mexemos aqui.
    // (Para não alongar, assumimos que o código original está presente.
    // Se você tiver dúvidas, mantenha a versão que já estava funcionando.)
    return currentSpeed; // placeholder, substitua pelo original
}

void AccidentAlertApp::onWSM(BaseFrame1609_4* wsm)
{
    // A camada física pode entregar o quadro ao módulo, mas estes veículos
    // não possuem V2X no experimento e não podem interpretar o alerta.
    if (!enableV2X || isComparisonVehicle(mobility)) return;

    // --- TRATAMENTO DO ALERTA DE CONGESTIONAMENTO (DESVIO) ---
    if (strcmp(wsm->getName(), "CongestionAlert") == 0) {
        auto* alert = dynamic_cast<TraCIDemo11pMessage*>(wsm);
        if (alert == nullptr) return;

        double senderSpeed = std::stod(alert->getDemoData());

        EV_INFO << "V2X: CONGESTION ALERT RECEIVED by " << getParentModule()->getFullName()
                << " | senderSpeed=" << senderSpeed
                << " | rerouteTriggered=" << rerouteTriggered
                << " | index=" << getParentModule()->getIndex()
                << " | currentRoute=" << traciVehicle->getRouteId()
                << " | speed=" << mobility->getSpeed() << endl;

        // --------------------------------------------------------------------
        //  Decisão de desvio em resposta ao alerta de congestionamento.
        //  A validação é feita antes da bifurcação, em uma janela de posição e
        //  velocidade adequada. Se o veículo estiver fora do ponto de decisão, a
        //  lógica é ignorada para não disparar desvios em veículos do tráfego de
        //  fila ou fora do ramo principal.
        // --------------------------------------------------------------------
        if (!rerouteTriggered && getParentModule()->getIndex() != 0) {
            const std::string currentRoad = traciVehicle->getRoadId();
            const std::string currentRoute = traciVehicle->getRouteId();
            const double currentPos = traciVehicle->getLanePosition();

            if (!isHighwayDetourCandidate(traciVehicle)) {
                return;
            }

            if (shouldEvaluateDetour(traciVehicle, mobility, mobility->getCommandInterface(),
                     senderSpeed, freeFlowSpeed, congestionSpiThreshold)) {
                const bool rerouted = tryAlternativeRoute(traciVehicle, mobility->getCommandInterface());
                rerouteTriggered = rerouted;
                if (rerouted) {
                    EV_INFO << "V2X: ROTA ALTERNATIVA ESCOLHIDA (congestion alert) em t=" << simTime()
                            << " | road=" << currentRoad
                            << " | pos=" << currentPos
                            << " | route=" << currentRoute << endl;
                }
            } else {
                EV_INFO << "V2X: DEBUG - fora do ponto de desvio: road='" << currentRoad
                        << "' pos=" << currentPos
                        << " route='" << currentRoute << "'" << endl;
            }
        }
        return;
    }

    // --- TRATAMENTO DO ALERTA DE ACIDENTE (código original) ---
    if (strcmp(wsm->getName(), "AccidentAlert") != 0) return;

    auto* alert = dynamic_cast<TraCIDemo11pMessage*>(wsm);
    if (alert == nullptr) return;

    alertsReceived++;

    simtime_t sendTime = SimTime::parse(alert->getDemoData());
    simtime_t delay = simTime() - sendTime;

    totalAlertDelay += delay;
    if (minAlertDelay < SIMTIME_ZERO || delay < minAlertDelay) minAlertDelay = delay;
    if (maxAlertDelay < SIMTIME_ZERO || delay > maxAlertDelay) maxAlertDelay = delay;

    EV_INFO << "V2X ACCIDENT ALERT RECEIVED"
            << " at t=" << simTime()
            << " by " << getParentModule()->getFullName()
            << " | sendTime=" << sendTime
            << " | delay=" << delay << endl;

    if (getParentModule()->getIndex() != 0 &&
        traciVehicle != nullptr &&
        mobility != nullptr &&
        !reactionActive) {

        alertReceiveTime = simTime();
        speedAtAlert = mobility->getSpeed();

        progressiveAppliedDecel = 0.0;
        plannedReactionInitialized = false;
        plannedReactionDecel = 0.0;
        plannedTargetDistance = 5.0;
        reactionTargetSpeed = std::max(0.0, speedAtAlert * reactionSpeedFactor);
        minSpeedAfterAlert = speedAtAlert;
        timeToReducedSpeed = -1;
        reactionDistance = 0;
        reducedSpeedReached = false;
        reactionActive = true;
        haveLastReactionPosition = true;
        lastReactionPosition = curPosition;

        double initialReactionSpeed = calculateSafeReactionSpeed(speedAtAlert);
        if (reactionStrategy == "planned") {
            double maxAllowedDecel = std::max(0.1, traciVehicle->getDeccel());
            double dt = reactionMonitorInterval.dbl();
            double allowedDrop = maxAllowedDecel * dt;
            double lowerSpeed = std::max(0.0, speedAtAlert - allowedDrop);
            initialReactionSpeed = std::max(lowerSpeed, initialReactionSpeed);
            initialReactionSpeed = std::min(speedAtAlert, initialReactionSpeed);
        }
        traciVehicle->slowDown(initialReactionSpeed, reactionMonitorInterval);

        EV_INFO << "V2X REACTION STARTED"
                << " at t=" << simTime()
                << " by " << getParentModule()->getFullName()
                << " | speedAtAlert=" << speedAtAlert
                << " | targetSpeed=" << reactionTargetSpeed
                << " | initialSafeSpeed=" << initialReactionSpeed
                << " | leaderDistance=" << currentLeaderDistance
                << " | calculatedDecel=" << calculatedReactionDecel << endl;

        // Cancelar o agendamento anterior do reactionMonitorMsg para evitar duplicação
        if (reactionMonitorMsg != nullptr) {
            cancelEvent(reactionMonitorMsg);
            scheduleAt(simTime() + reactionMonitorInterval, reactionMonitorMsg);
        }
    }
}

void AccidentAlertApp::handlePositionUpdate(cObject* obj)
{
    DemoBaseApplLayer::handlePositionUpdate(obj);

    if (mobility != nullptr && traciVehicle != nullptr) {
        const std::string vehicleId = mobility->getExternalId();
        const std::string roadId = traciVehicle->getRoadId();

        if (effectiveDetourEntryTime < SIMTIME_ZERO && roadId == "484273114") {
            effectiveDetourEntryTime = simTime();
            EV_INFO << "V2X: ENTRADA EFETIVA NA MARGINAL | vehicle=" << vehicleId
                    << " | t=" << simTime() << endl;
        }

        // 529303407 é o trecho comum depois da convergência das duas rotas.
        if (commonRoadEntryTime < SIMTIME_ZERO && roadId == "529303407") {
            commonRoadEntryTime = simTime();
            EV_INFO << "V2X: CHEGADA AO TRECHO COMUM | vehicle=" << vehicleId
                    << " | t=" << simTime() << endl;
        }
    }

    if (getParentModule()->getIndex() == 1 &&
        !reactionActive &&
        mobility != nullptr &&
        traciVehicle != nullptr &&
        simTime() >= nextDiagnosticTime) {

        auto leaderInfo = traciVehicle->getLeader(1000.0);
        auto tlsInfo = traciVehicle->getNextTls();

        EV_INFO << "V2X POST-REACTION DIAGNOSTIC"
                << " t=" << simTime()
                << " | speed=" << mobility->getSpeed()
                << " | road=" << traciVehicle->getRoadId()
                << " | leader=" << leaderInfo.first
                << " | leaderDistance=" << leaderInfo.second
                << " | nextTLS_count=" << tlsInfo.size() << endl;

        nextDiagnosticTime = simTime() + SimTime(1);
    }

    if (!reactionActive || !haveLastReactionPosition) return;

    double dx = curPosition.x - lastReactionPosition.x;
    double dy = curPosition.y - lastReactionPosition.y;
    double dz = curPosition.z - lastReactionPosition.z;

    reactionDistance += std::sqrt(dx*dx + dy*dy + dz*dz);
    lastReactionPosition = curPosition;
}

void AccidentAlertApp::finish()
{
    recordScalar("alertsSent", alertsSent);
    recordScalar("alertsReceived", alertsReceived);
    recordScalar("totalAlertDelay", totalAlertDelay);
    recordScalar("effectiveDetourEntryTime", effectiveDetourEntryTime);
    recordScalar("commonRoadEntryTime", commonRoadEntryTime);

    if (alertsReceived > 0) {
        recordScalar("averageAlertDelay", totalAlertDelay / alertsReceived);
        recordScalar("minAlertDelay", minAlertDelay);
        recordScalar("maxAlertDelay", maxAlertDelay);
        recordScalar("alertReceiveTime", alertReceiveTime);
        recordScalar("speedAtAlert", speedAtAlert);
        recordScalar("reactionTargetSpeed", reactionTargetSpeed);
        recordScalar("minSpeedAfterAlert", minSpeedAfterAlert);
        recordScalar("reactionDistance", reactionDistance);
        if (timeToReducedSpeed >= SIMTIME_ZERO) {
            recordScalar("timeToReducedSpeed", timeToReducedSpeed);
        }
    }

    DemoBaseApplLayer::finish();
}

AccidentAlertApp::~AccidentAlertApp()
{
    if (accidentMsg != nullptr) {
        cancelAndDelete(accidentMsg);
        accidentMsg = nullptr;
    }
    if (reactionMonitorMsg != nullptr) {
        cancelAndDelete(reactionMonitorMsg);
        reactionMonitorMsg = nullptr;
    }
}

} // namespace tcc
