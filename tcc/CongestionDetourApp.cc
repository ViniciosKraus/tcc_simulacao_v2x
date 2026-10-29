#include "CongestionDetourApp.h"

#include <veins/modules/application/traci/TraCIDemo11pMessage_m.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <list>

using namespace omnetpp;
using namespace veins;

namespace {
constexpr double ALERT_METRIC_BEGIN = 60.0;

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
        "529303430.431",
        "529303430.631",
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
    veins::TraCICommandInterface* commandInterface)
{
    if (traciVehicle == nullptr || commandInterface == nullptr) {
        throw cRuntimeError("Cannot estimate free-flow speed without a vehicle and TraCI interface");
    }

    const std::string laneId = traciVehicle->getLaneId();
    if (laneId.empty()) {
        throw cRuntimeError("Cannot estimate free-flow speed: vehicle has no current lane");
    }

    double laneMaxSpeed;
    try {
        auto lane = commandInterface->lane(laneId);
        laneMaxSpeed = lane.getMaxSpeed();
    } catch (const std::exception& e) {
        throw cRuntimeError("Cannot read speed limit for current lane %s: %s",
            laneId.c_str(), e.what());
    }

    if (laneMaxSpeed <= 0.0) {
        throw cRuntimeError("Invalid speed limit for current lane %s: %g",
            laneId.c_str(), laneMaxSpeed);
    }

    return laneMaxSpeed;
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

    return alternativeCost < currentCost * 0.75;
}

bool shouldEvaluateDetour(veins::TraCICommandInterface::Vehicle* traciVehicle,
    const veins::TraCIMobility* mobility,
    veins::TraCICommandInterface* commandInterface,
    double senderSpi,
    double congestionSpiThreshold = 50.0)
{
    if (traciVehicle == nullptr || mobility == nullptr || !isHighwayDetourCandidate(traciVehicle)) {
        return false;
    }

    const std::string currentRoad = traciVehicle->getRoadId();
    const std::string currentRoute = traciVehicle->getRouteId();
    const double currentPos = traciVehicle->getLanePosition();
    const double currentSpeed = mobility->getSpeed();
    const auto plannedRoads = traciVehicle->getPlannedRoadIds();
    const bool beforeBranch = isOnHighwayApproach(traciVehicle, 200.0);
    const bool congestionAlert = senderSpi < congestionSpiThreshold;
    const bool stillOnMainPath = currentRoad == "529303429";
    const bool worthwhileDetour = detourIsWorthwhile(traciVehicle, commandInterface, currentSpeed);
    const bool shouldEvaluate = beforeBranch && stillOnMainPath && worthwhileDetour && congestionAlert;

    EV_INFO << "V2X: CHECK_DETOUR | node=" << (mobility ? mobility->getExternalId() : "?")
            << " | road=" << currentRoad
            << " pos=" << currentPos
            << " route=" << currentRoute
            << " speed=" << currentSpeed
            << " senderSpi=" << senderSpi
            << " beforeBranch=" << beforeBranch
            << " congestionAlert=" << congestionAlert
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

Define_Module(tcc::CongestionDetourApp);

namespace tcc {

void CongestionDetourApp::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);

    if (stage == 0) {

        EV_INFO << "V2X: NODE MAP | omnetIndex=" << getParentModule()->getIndex()
                << " | externalId=" << mobility->getExternalId()
                << " | initialRoute=" << traciVehicle->getRouteId() << endl;

        enableV2X = par("enableV2X").boolValue();
        vehicleTypeId = traciVehicle->getVType();
        if (par("useVehicleTypeForV2X").boolValue()) {
            enableV2X = enableV2X && vehicleTypeId == "car_v2x";
        }
        congestionMonitorInterval = par("congestionMonitorInterval");

        congestionSpiThreshold = par("congestionSpiThreshold").doubleValue();
        congestionSpiPersistenceChecks = par("congestionSpiPersistenceChecks").intValue();
        if (congestionSpiPersistenceChecks <= 0) {
            throw cRuntimeError("congestionSpiPersistenceChecks must be greater than zero");
        }
        if (getParentModule()->getIndex() != 0 && enableV2X) {
            congestionMonitorMsg = new cMessage("congestionMonitor");
            scheduleAt(simTime() + congestionMonitorInterval, congestionMonitorMsg);
        }
    }
    rerouteTriggered = false;
    rerouteTriggered = false;
    lastCongestionAlertTime = simTime();
}

void CongestionDetourApp::handleSelfMsg(cMessage* msg)
{
    // Monitoramento periódico do tráfego para emissão de alertas.
    if (msg == congestionMonitorMsg) {

        if (enableV2X && getParentModule()->getIndex() != 0) {
            const double speed = mobility->getSpeed();
            const double effectiveFreeFlowSpeed = estimateFreeFlowSpeed(
                traciVehicle, mobility->getCommandInterface());
            const double spi = computeSpeedPerformanceIndex(speed, effectiveFreeFlowSpeed);
            if (spi < congestionSpiThreshold) {
                if (lowSpiConsecutiveChecks < congestionSpiPersistenceChecks) {
                    ++lowSpiConsecutiveChecks;
                }
            } else {
                lowSpiConsecutiveChecks = 0;
            }

            if (lowSpiConsecutiveChecks >= congestionSpiPersistenceChecks &&
                simTime() - lastCongestionAlertTime > 2.0) {
                auto* wsm = new TraCIDemo11pMessage();
                wsm->setName("CongestionAlert");
                populateWSM(wsm);
                wsm->setSenderAddress(myId);
                wsm->setSerial(++alertSequence);
                char spiStr[16];
                std::snprintf(spiStr, sizeof(spiStr), "%.2f", spi);
                wsm->setDemoData(spiStr);
                if (simTime().dbl() >= ALERT_METRIC_BEGIN) {
                    ++alertsSent;
                }
                sendDown(wsm);
                lastCongestionAlertTime = simTime();
                EV_INFO << "V2X: CONGESTION ALERT SENT by "
                        << getParentModule()->getFullName()
                        << " | speed=" << speed
                        << " | spi=" << spi
                        << " | threshold=" << congestionSpiThreshold
                        << " | consecutiveChecks=" << lowSpiConsecutiveChecks << endl;
            }
        }

        scheduleAt(simTime() + congestionMonitorInterval, congestionMonitorMsg);
        return;
    }

    DemoBaseApplLayer::handleSelfMsg(msg);
}

void CongestionDetourApp::onWSM(BaseFrame1609_4* wsm)
{
    if (!enableV2X) return;

    if (strcmp(wsm->getName(), "CongestionAlert") != 0) return;

    auto* alert = dynamic_cast<TraCIDemo11pMessage*>(wsm);
    if (alert == nullptr) return;

    if (simTime().dbl() >= ALERT_METRIC_BEGIN &&
        alert->getCreationTime().dbl() >= ALERT_METRIC_BEGIN &&
        alert->getSenderAddress() >= 0 && alert->getSerial() > 0) {
        ++alertsReceived;
        const simtime_t latency = simTime() - alert->getCreationTime();
        if (latency >= SIMTIME_ZERO) {
            alertLatencySum += latency.dbl();
            ++alertLatencyCount;
        }
    }

    const double senderSpi = std::stod(alert->getDemoData());

    if (rerouteTriggered || getParentModule()->getIndex() == 0 ||
        !isHighwayDetourCandidate(traciVehicle)) {
        return;
    }

    const std::string currentRoad = traciVehicle->getRoadId();
    const std::string currentRoute = traciVehicle->getRouteId();
    const double currentPos = traciVehicle->getLanePosition();

    if (shouldEvaluateDetour(traciVehicle, mobility, mobility->getCommandInterface(),
            senderSpi, congestionSpiThreshold)) {
        const bool rerouted = tryAlternativeRoute(traciVehicle, mobility->getCommandInterface());
        rerouteTriggered = rerouted;
        if (rerouted) {
            alertTriggeredReroute = true;
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

void CongestionDetourApp::handlePositionUpdate(cObject* obj)
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
}

void CongestionDetourApp::finish()
{
    recordScalar("vehicleHasV2X", enableV2X ? 1 : 0);
    recordScalar("vehicleIsCarV2X", vehicleTypeId == "car_v2x" ? 1 : 0);
    recordScalar("vehicleIsCarNormal", vehicleTypeId == "car_normal" ? 1 : 0);
    recordScalar("vehicleIsTruck", vehicleTypeId == "truck_heavy" ? 1 : 0);
    recordScalar("effectiveDetourEntryTime", effectiveDetourEntryTime);
    recordScalar("commonRoadEntryTime", commonRoadEntryTime);
    recordScalar("alertsSent", alertsSent);
    recordScalar("alertsReceived", alertsReceived);
    recordScalar("alertLatencySum", alertLatencySum);
    recordScalar("alertLatencyCount", alertLatencyCount);
    recordScalar("alertTriggeredReroute", alertTriggeredReroute ? 1 : 0);
    recordScalar("reroutesRequireReceivedAlert", 1);

    DemoBaseApplLayer::finish();
}

CongestionDetourApp::~CongestionDetourApp()
{
    if (congestionMonitorMsg != nullptr) {
        cancelAndDelete(congestionMonitorMsg);
        congestionMonitorMsg = nullptr;
    }
}

} // namespace tcc
