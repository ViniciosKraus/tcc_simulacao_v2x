#include "AccidentAlertApp.h"

#include <veins/modules/application/traci/TraCIDemo11pMessage_m.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace omnetpp;
using namespace veins;

namespace tcc {

Define_Module(AccidentAlertApp);

void AccidentAlertApp::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);

    if (stage == 0) {

        enableV2X = par("enableV2X").boolValue();
        reactionStrategy = par("reactionStrategy").stdstringValue();
        reactionSpeedFactor = par("reactionSpeedFactor").doubleValue();

        /*
         * Carrega os parametros definidos no omnetpp.ini.
         * Dessa forma, cada cenario pode configurar sua propria
         * estrategia de reacao e seus parametros de controle.
         */
        safeDistance = par("safeDistance").doubleValue();

        reactionDuration = par("reactionDuration");
        reactionMonitorInterval = par("reactionMonitorInterval");

        /*
         * Somente o primeiro veiculo inicia a transmissao do
         * alerta de acidente. No Scenario A, o acidente continua
         * ocorrendo, mas a comunicacao V2X permanece desativada.
         */
        if (getParentModule()->getIndex() == 0 && enableV2X) {
            accidentMsg = new cMessage("accidentAlert");
            scheduleAt(20.1, accidentMsg);
        }

        /*
         * Veiculos receptores criam o temporizador usado para
         * atualizar periodicamente o controle da reacao.
         */
        if (getParentModule()->getIndex() != 0 && enableV2X) {
            reactionMonitorMsg = new cMessage("reactionMonitor");
        }
    }
}

void AccidentAlertApp::handleSelfMsg(cMessage* msg)
{
    if (msg == accidentMsg && !accidentTriggered) {

        accidentTriggered = true;

        auto* wsm = new TraCIDemo11pMessage();
        wsm->setName("AccidentAlert");

        populateWSM(wsm);
        wsm->setSerial(1);

        char sendTimeString[64];

        std::snprintf(
            sendTimeString,
            sizeof(sendTimeString),
            "%.12f",
            simTime().dbl()
        );

        wsm->setDemoData(sendTimeString);

        sendDown(wsm);

        alertsSent++;

        EV_INFO << "V2X ACCIDENT ALERT SENT"
                << " at t=" << simTime()
                << " by "
                << getParentModule()->getFullName()
                << endl;
    }
    else if (msg == reactionMonitorMsg && reactionActive) {

        /*
         * Ao atingir o tempo minimo configurado, inicia a etapa
         * de verificacao para decidir quando o controle pode
         * retornar ao comportamento normal do SUMO.
         */
        if (simTime() - alertReceiveTime >= reactionDuration) {

            /*
             * A reacao nao termina automaticamente apenas porque
             * o tempo configurado foi atingido. Se o lider ainda
             * estiver parado, V2 permanece parado para evitar
             * uma nova aproximacao do veiculo acidentado.
             *
             * O controle normal do SUMO e liberado quando o lider
             * volta a se mover.
             */
            auto leaderInfo =
                traciVehicle->getLeader(1000.0);

            if (!leaderInfo.first.empty() &&
                leaderInfo.second >= 0.0) {

                auto leaderVehicle =
                    mobility->getCommandInterface()
                        ->vehicle(leaderInfo.first);

                double leaderSpeed =
                    leaderVehicle.getSpeed();

                /*
                 * O lider ainda esta parado. V2 permanece parado
                 * e o monitoramento continua ativo.
                 */
                if (leaderSpeed <= 0.5) {

                    traciVehicle->setSpeed(0);

                    EV_INFO << "V2X WAITING FOR LEADER"
                            << " | t=" << simTime()
                            << " | leader="
                            << leaderInfo.first
                            << " | leaderSpeed="
                            << leaderSpeed
                            << " | leaderDistance="
                            << leaderInfo.second
                            << endl;

                    scheduleAt(
                        simTime() + reactionMonitorInterval,
                        reactionMonitorMsg
                    );

                    return;
                }

                /*
                 * O lider voltou a se mover. A reacao V2X termina
                 * e o controle de velocidade retorna ao SUMO.
                 */
                reactionActive = false;
                haveLastReactionPosition = false;

                double speedBeforeRelease =
                    mobility->getSpeed();

                traciVehicle->setSpeed(-1);

                EV_INFO << "V2X REACTION FINISHED"
                        << " | speedBeforeRelease="
                        << speedBeforeRelease
                        << " at t=" << simTime()
                        << " by "
                        << getParentModule()->getFullName()
                        << " | duration=" << reactionDuration
                        << " | leaderSpeed="
                        << leaderSpeed
                        << " | leaderDistance="
                        << leaderInfo.second
                        << " | reactionDistance="
                        << reactionDistance
                        << endl;

                return;
            }

            /*
             * Sem um lider identificado, V2 permanece parado
             * e continua monitorando a situacao.
             */
            traciVehicle->setSpeed(0);

            scheduleAt(
                simTime() + reactionMonitorInterval,
                reactionMonitorMsg
            );

            return;
        }

        if (mobility == nullptr || traciVehicle == nullptr) {
            return;
        }

        double currentSpeed = mobility->getSpeed();

        if (minSpeedAfterAlert < 0 ||
            currentSpeed < minSpeedAfterAlert) {

            minSpeedAfterAlert = currentSpeed;
        }

        /*
         * Registra o instante em que a velocidade de reacao
         * e atingida pela primeira vez.
         */
        if (!reducedSpeedReached &&
            currentSpeed <= reactionTargetSpeed + 0.1) {

            reducedSpeedReached = true;
            timeToReducedSpeed =
                simTime() - alertReceiveTime;

            EV_INFO << "V2X REACTION TARGET REACHED"
                    << " at t=" << simTime()
                    << " by "
                    << getParentModule()->getFullName()
                    << " | targetSpeed=" << reactionTargetSpeed
                    << " | currentSpeed=" << currentSpeed
                    << " | timeToTarget=" << timeToReducedSpeed
                    << endl;
        }

            /*
             * Recalcula a velocidade de controle com base na
             * situacao atual do V2 e na distancia ate o lider.
             */
            double safeReactionSpeed =
                calculateSafeReactionSpeed(currentSpeed);

                    /*
                     * Na estrategia "current", preservamos a referencia
                     * de velocidade definida no instante do alerta
                     * quando nao ha necessidade de uma nova reducao.
                     */
                    if (reactionStrategy == "current") {

                        if (currentLeaderDistance < 0.0) {
                            safeReactionSpeed = speedAtAlert;
                        }

                        if (currentLeaderDistance > safeDistance &&
                            calculatedReactionDecel <= 0.0) {

                            safeReactionSpeed = speedAtAlert;
                        }
                    }

                    /*
                     * Na estrategia "planned", a velocidade calculada
                     * e limitada pela desaceleracao maxima configurada
                     * para o veiculo no SUMO. O limite e aplicado por
                     * intervalo de controle para evitar uma variacao
                     * de velocidade maior que a permitida.
                     */
                    if (reactionStrategy == "planned") {

                        double maxAllowedDecel =
                            std::max(
                                0.1,
                                traciVehicle->getDeccel()
                            );

                        double dt =
                            reactionMonitorInterval.dbl();

                        double allowedDrop =
                            maxAllowedDecel * dt;

                        double lowerSpeed =
                            std::max(
                                0.0,
                                currentSpeed - allowedDrop
                            );

                        /*
                         * Restringe a variacao de velocidade ao limite
                         * de desaceleracao permitido neste intervalo.
                         */
                        safeReactionSpeed =
                            std::max(
                                lowerSpeed,
                                safeReactionSpeed
                            );

                        /*
                         * Durante a reacao planejada, a velocidade
                         * nao pode aumentar artificialmente.
                         */
                        safeReactionSpeed =
                            std::min(
                                currentSpeed,
                                safeReactionSpeed
                            );
                    }

                    traciVehicle->slowDown(
                        safeReactionSpeed,
                        reactionMonitorInterval
                    );

        EV_INFO << "V2X CONTROL"
                << " at t=" << simTime()
                << " | currentSpeed=" << currentSpeed
                << " | safeSpeed=" << safeReactionSpeed
                << " | leaderDistance=" << currentLeaderDistance
                << " | decel=" << calculatedReactionDecel
                << endl;

        /*
         * Mantem o controle da reacao enquanto ela estiver ativa.
         * A velocidade e reavaliada a cada intervalo configurado.
         */
        scheduleAt(
            simTime() + reactionMonitorInterval,
            reactionMonitorMsg
        );
    }
    else {
        DemoBaseApplLayer::handleSelfMsg(msg);
    }
}

double AccidentAlertApp::calculateSafeReactionSpeed(double currentSpeed)
{
    if (traciVehicle == nullptr || mobility == nullptr) {
        return currentSpeed;
    }

    auto leaderInfo = traciVehicle->getLeader(1000.0);

    const std::string& leaderId = leaderInfo.first;
    double leaderDistance = leaderInfo.second;

    if (leaderId.empty() || leaderDistance < 0.0) {
        currentLeaderDistance = -1.0;
        calculatedReactionDecel = 0.0;
        return currentSpeed;
    }

    currentLeaderDistance = leaderDistance;

    /*
     * Scenario B - estrategia "current".
     *
     * A velocidade-alvo e definida a partir da velocidade
     * observada no instante do alerta. A desaceleracao necessaria
     * e calculada em funcao da distancia disponivel ate o lider.
     */
    if (reactionStrategy == "current") {

        double availableDistance =
            leaderDistance - safeDistance;

        double vehicleDecel =
            std::max(
                0.1,
                traciVehicle->getDeccel()
            );

        if (availableDistance <= 0.0) {
            calculatedReactionDecel = vehicleDecel;
            return 0.0;
        }

        double targetSpeed = reactionTargetSpeed;

        double requiredDecel =
            (targetSpeed * targetSpeed -
             currentSpeed * currentSpeed) /
            (2.0 * availableDistance);

        if (requiredDecel >= 0.0) {
            calculatedReactionDecel = 0.0;
            return currentSpeed;
        }

        double decelMagnitude =
            std::min(
                std::abs(requiredDecel),
                vehicleDecel
            );

        calculatedReactionDecel = decelMagnitude;

        double safeSpeedSquared =
            currentSpeed * currentSpeed -
            2.0 * decelMagnitude * availableDistance;

        double safeSpeed =
            std::sqrt(
                std::max(0.0, safeSpeedSquared)
            );

        return std::min(currentSpeed, safeSpeed);
    }

    /*
     * Scenario C - estrategia "progressive".
     *
     * A frenagem e distribuida de forma progressiva usando
     * a distancia de controle e a velocidade do lider como
     * referencias para evitar uma reducao excessivamente brusca.
     */
    if (reactionStrategy == "progressive") {

        double availableDistance =
            leaderDistance - safeDistance;

        double anticipationDistance = 5.0;

        double controlDistance =
            availableDistance + anticipationDistance;

        double vehicleDecel =
            std::max(
                0.1,
                traciVehicle->getDeccel()
            );

        if (availableDistance <= 0.0) {

            calculatedReactionDecel =
                currentSpeed > 0.0
                    ? vehicleDecel
                    : 0.0;

            return 0.0;
        }

        auto leaderVehicle =
            mobility->getCommandInterface()->vehicle(leaderId);

        double leaderSpeed =
            std::max(0.0, leaderVehicle.getSpeed());

        double desiredSpeed =
            std::max(
                reactionTargetSpeed,
                leaderSpeed
            );

        if (currentSpeed <= desiredSpeed + 0.01) {

            calculatedReactionDecel = 0.0;

            return desiredSpeed;
        }

        double requiredBrakingDistance =
            (
                currentSpeed * currentSpeed -
                desiredSpeed * desiredSpeed
            ) /
            (2.0 * vehicleDecel);

        if (requiredBrakingDistance <= controlDistance) {

            double requiredDecel =
                (
                    currentSpeed * currentSpeed -
                    desiredSpeed * desiredSpeed
                ) /
                (2.0 * controlDistance);

            progressiveAppliedDecel =
                std::min(
                    std::max(0.0, requiredDecel),
                    vehicleDecel
                );

            calculatedReactionDecel =
                progressiveAppliedDecel;

            double dt =
                reactionMonitorInterval.dbl();

            double targetSpeed =
                currentSpeed -
                progressiveAppliedDecel * dt;

            return std::max(
                desiredSpeed,
                targetSpeed
            );
        }

        progressiveAppliedDecel =
            vehicleDecel;

        calculatedReactionDecel =
            progressiveAppliedDecel;

        double dt =
            reactionMonitorInterval.dbl();

        double targetSpeed =
            currentSpeed -
            progressiveAppliedDecel * dt;

        return std::max(
            0.0,
            targetSpeed
        );
    }

    /*
     * Scenario D - estrategia "planned".
     *
     * A cada ciclo de controle, a desaceleracao necessaria e
     * recalculada usando:
     *
     *   - velocidade atual do V2;
     *   - velocidade atual do lider;
     *   - distancia atual entre os veiculos;
     *   - distancia-alvo de 5 m.
     *
     * O objetivo e reduzir a velocidade relativa para que o V2
     * se aproxime do lider de forma controlada. A desaceleracao
     * calculada e limitada pelo valor permitido pelo veiculo.
     */

    if (reactionStrategy == "planned") {

        auto leaderVehicle =
            mobility->getCommandInterface()->vehicle(leaderId);

        double leaderSpeed =
            std::max(0.0, leaderVehicle.getSpeed());

        double controlDistance =
            leaderDistance - plannedTargetDistance;

        double maxAllowedDecel =
            std::max(
                0.1,
                traciVehicle->getDeccel()
            );

        /*
         * Se a distancia-alvo ja foi atingida, a velocidade
         * e reduzida de forma progressiva para evitar uma parada
         * instantanea.
         */
        if (controlDistance <= 0.0) {

            calculatedReactionDecel =
                currentSpeed > 0.0
                    ? maxAllowedDecel
                    : 0.0;

            double dt =
                reactionMonitorInterval.dbl();

            double targetSpeed =
                std::max(
                    0.0,
                    currentSpeed -
                    maxAllowedDecel * dt
                );

            return targetSpeed;
        }

        /*
         * Se V2 ja esta na velocidade do lider ou abaixo dela,
         * nao ha necessidade de continuar desacelerando.
         */
        if (currentSpeed <= leaderSpeed) {

            plannedReactionDecel = 0.0;
            calculatedReactionDecel = 0.0;

            return currentSpeed;
        }

        /*
         * Desaceleracao necessaria para reduzir a velocidade do V2
         * ate aproximadamente a velocidade do lider na distancia
         * restante.
         *
         * Equacao de movimento:
         *
         *   vf^2 = vi^2 - 2*a*d
         *
         * Portanto:
         *
         * a = (v2^2 - v1^2) / (2*d)
         */
        double requiredDecel =
            (
                currentSpeed * currentSpeed -
                leaderSpeed * leaderSpeed
            ) /
            (
                2.0 * controlDistance
            );

        requiredDecel =
            std::max(
                0.0,
                requiredDecel
            );

        /*
         * A desaceleracao e recalculada dinamicamente.
         * O limite do veiculo atua apenas como restricao maxima;
         * ele nao determina a desaceleracao aplicada em todos
         * os ciclos.
         */
        plannedReactionDecel =
            std::min(
                requiredDecel,
                maxAllowedDecel
            );

        calculatedReactionDecel =
            plannedReactionDecel;

        /*
         * Converte a desaceleracao calculada em uma variacao
         * de velocidade para o proximo intervalo de controle.
         */
        double dt =
            reactionMonitorInterval.dbl();

        double maxSpeedDrop =
            plannedReactionDecel * dt;

        double targetSpeed =
            std::max(
                leaderSpeed,
                currentSpeed - maxSpeedDrop
            );

        EV_INFO
            << "V2X DYNAMIC PLANNING"
            << " | t=" << simTime()
            << " | currentSpeed=" << currentSpeed
            << " | leaderSpeed=" << leaderSpeed
            << " | leaderDistance=" << leaderDistance
            << " | targetDistance=" << plannedTargetDistance
            << " | requiredDecel=" << requiredDecel
            << " | appliedDecel=" << plannedReactionDecel
            << " | maxAllowedDecel=" << maxAllowedDecel
            << endl;

        return targetSpeed;
    }

    return currentSpeed;
}

void AccidentAlertApp::onWSM(BaseFrame1609_4* wsm)
{
    if (!enableV2X) {
        return;
    }

    if (strcmp(wsm->getName(), "AccidentAlert") != 0) {
        return;
    }

    auto* alert = dynamic_cast<TraCIDemo11pMessage*>(wsm);

    if (alert == nullptr) {
        return;
    }

    alertsReceived++;

    simtime_t sendTime = SimTime::parse(
        alert->getDemoData()
    );

    simtime_t delay = simTime() - sendTime;

    totalAlertDelay += delay;

    if (minAlertDelay < SIMTIME_ZERO || delay < minAlertDelay) {
        minAlertDelay = delay;
    }

    if (maxAlertDelay < SIMTIME_ZERO || delay > maxAlertDelay) {
        maxAlertDelay = delay;
    }

    EV_INFO << "V2X ACCIDENT ALERT RECEIVED"
            << " at t=" << simTime()
            << " by "
            << getParentModule()->getFullName()
            << " | sendTime=" << sendTime
            << " | delay=" << delay
            << endl;

    /*
     * O veiculo receptor inicia a reacao V2X quando possui
     * os recursos de mobilidade e comunicacao disponiveis.
     */
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

        reactionTargetSpeed =
            std::max(0.0, speedAtAlert * reactionSpeedFactor);

        minSpeedAfterAlert = speedAtAlert;
        timeToReducedSpeed = -1;
        reactionDistance = 0;

        // Reinicia os estados internos da estrategia planejada.
        plannedReactionInitialized = false;
        plannedReactionDecel = 0.0;
        plannedTargetDistance = 5.0;

        reducedSpeedReached = false;
        reactionActive = true;
        haveLastReactionPosition = true;
        lastReactionPosition = curPosition;

        /*
         * Define a primeira velocidade de reacao a partir
         * da situacao atual do veiculo e do lider.
         */
        double initialReactionSpeed =
            calculateSafeReactionSpeed(speedAtAlert);

        /*
         * Na estrategia planejada, a primeira aplicacao tambem
         * respeita o limite de desaceleracao do veiculo por
         * intervalo de controle. Esse limite funciona como
         * restricao de seguranca, nao como valor fixo de frenagem.
         */
        if (reactionStrategy == "planned") {

            double maxAllowedDecel =
                std::max(
                    0.1,
                    traciVehicle->getDeccel()
                );

            double dt =
                reactionMonitorInterval.dbl();

            double allowedDrop =
                maxAllowedDecel * dt;

            double lowerSpeed =
                std::max(
                    0.0,
                    speedAtAlert - allowedDrop
                );

            initialReactionSpeed =
                std::max(
                    lowerSpeed,
                    initialReactionSpeed
                );

            initialReactionSpeed =
                std::min(
                    speedAtAlert,
                    initialReactionSpeed
                );
        }

        traciVehicle->slowDown(
            initialReactionSpeed,
            reactionMonitorInterval
        );

        EV_INFO << "V2X REACTION STARTED"
                << " at t=" << simTime()
                << " by "
                << getParentModule()->getFullName()
                << " | speedAtAlert=" << speedAtAlert
                << " | targetSpeed=" << reactionTargetSpeed
                << " | initialSafeSpeed=" << initialReactionSpeed
                << " | leaderDistance=" << currentLeaderDistance
                << " | calculatedDecel=" << calculatedReactionDecel
                << endl;

        if (reactionMonitorMsg != nullptr) {
            scheduleAt(
                simTime() + reactionMonitorInterval,
                reactionMonitorMsg
            );
        }
    }
}

void AccidentAlertApp::handlePositionUpdate(cObject* obj)
{
    DemoBaseApplLayer::handlePositionUpdate(obj);

    /*
     * Acompanha o comportamento do V2 apos a reacao, permitindo
     * verificar a retomada da velocidade, a distancia para o lider
     * e a continuidade da rota. O diagnostico e registrado no
     * maximo uma vez por segundo.
     */
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
                << " | nextTLS_count=" << tlsInfo.size()
                << endl;

        nextDiagnosticTime = simTime() + SimTime(1);
    }

    if (!reactionActive || !haveLastReactionPosition) {
        return;
    }

    double dx = curPosition.x - lastReactionPosition.x;
    double dy = curPosition.y - lastReactionPosition.y;
    double dz = curPosition.z - lastReactionPosition.z;

    reactionDistance += std::sqrt(
        dx * dx +
        dy * dy +
        dz * dz
    );

    lastReactionPosition = curPosition;
}

void AccidentAlertApp::finish()
{
    recordScalar("alertsSent", alertsSent);
    recordScalar("alertsReceived", alertsReceived);

    recordScalar("totalAlertDelay", totalAlertDelay);

    if (alertsReceived > 0) {

        recordScalar(
            "averageAlertDelay",
            totalAlertDelay / alertsReceived
        );

        recordScalar("minAlertDelay", minAlertDelay);
        recordScalar("maxAlertDelay", maxAlertDelay);

        recordScalar("alertReceiveTime", alertReceiveTime);
        recordScalar("speedAtAlert", speedAtAlert);
        recordScalar("reactionTargetSpeed", reactionTargetSpeed);
        recordScalar("minSpeedAfterAlert", minSpeedAfterAlert);
        recordScalar("reactionDistance", reactionDistance);

        if (timeToReducedSpeed >= SIMTIME_ZERO) {
            recordScalar(
                "timeToReducedSpeed",
                timeToReducedSpeed
            );
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

}