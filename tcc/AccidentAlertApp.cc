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
         * Carrega os parametros configurados no omnetpp.ini.
         * Isso permite que ScenarioB use 10 m e ScenarioC use 5 m.
         */
        safeDistance = par("safeDistance").doubleValue();

        reactionDuration = par("reactionDuration");
        reactionMonitorInterval = par("reactionMonitorInterval");

        /*
         * Apenas o primeiro veiculo gera o alerta.
         * No ScenarioA, o acidente continua existindo,
         * mas nenhuma mensagem V2X e criada.
         */
        if (getParentModule()->getIndex() == 0 && enableV2X) {
            accidentMsg = new cMessage("accidentAlert");
            scheduleAt(20.1, accidentMsg);
        }

        /*
         * Somente veiculos que podem receber o alerta
         * precisam monitorar uma possivel reacao.
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
         * Encerra a reacao apos o tempo configurado.
         * A partir daqui o veiculo volta ao controle normal do SUMO.
         */
        if (simTime() - alertReceiveTime >= reactionDuration) {

            /*
             * Apos o tempo minimo de reacao, V2 NAO devolve
             * imediatamente o controle ao SUMO.
             *
             * Enquanto V1 estiver parado, V2 permanece parado
             * devido a reacao V2X.
             *
             * O SUMO somente reassume quando o veiculo lider
             * voltar a se mover.
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
                 * V1 ainda esta parado:
                 * mantenha V2 parado e continue monitorando.
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
                 * V1 voltou a se mover.
                 * Agora o SUMO pode retomar o controle normal
                 * e V2 volta a seguir sua rota.
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
             * Se nao houver lider identificado, por seguranca
             * V2 permanece parado e continua monitorando.
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
        * Verifica se a velocidade alvo foi atingida.
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
            * Recalcula continuamente a velocidade segura
            * utilizando a distancia atual ate o lider.
            */
            double safeReactionSpeed =
                calculateSafeReactionSpeed(currentSpeed);

                    /*
                    * Essas regras pertencem apenas a estrategia atual.
                    * A estrategia progressive controla diretamente
                    * a velocidade em cada ciclo.
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
                     * Para o Scenario D, o calculo de
                     * calculateSafeReactionSpeed() define a
                     * desaceleracao dinamica.
                     *
                     * Aqui limitamos apenas a variacao de
                     * velocidade deste intervalo.
                     *
                     * O limite vem de maxReactionDecel e nao
                     * representa a desaceleracao normal.
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
                         * Nunca pede ao SUMO uma velocidade
                         * menor do que a permitida pelo limite
                         * de desaceleracao deste ciclo.
                         */
                        safeReactionSpeed =
                            std::max(
                                lowerSpeed,
                                safeReactionSpeed
                            );

                        /*
                         * Nunca aumenta artificialmente a
                         * velocidade durante a frenagem.
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
        * Continua o controle enquanto a reacao estiver ativa.
        *
        * O controle e atualizado a cada 0,1 s.
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
     * =========================================================
     * SCENARIO B - ESTRATEGIA CURRENT
     * =========================================================
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
     * =========================================================
     * SCENARIO C - ESTRATEGIA PROGRESSIVE
     * =========================================================
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
     * =========================================================
     * SCENARIO D - ESTRATEGIA PLANNED
     * =========================================================
     *
     * O objetivo é levar o V2 para aproximadamente 5 m atrás
     * do V1, considerando desde o instante do alerta:
     *
     *   - velocidade atual do V2;
     *   - velocidade atual do lider;
     *   - distancia atual;
     *   - distancia final desejada.
     *
     * Depois que o plano inicial é calculado, a cada ciclo fazemos
     * uma pequena correção baseada no erro de distância restante.
     *
     * O valor planejado de desaceleração funciona como referência
     * e a correção aumenta somente quando o V2 se aproxima demais
     * do lider.
     */
    /*
     * =========================================================
     * SCENARIO D - FRENAGEM PLANEJADA E SUAVIZADA
     * =========================================================
     *
     * No instante em que o alerta chega:
     *
     *   1. mede velocidade do V2;
     *   2. mede velocidade do lider;
     *   3. mede a distancia atual;
     *   4. calcula a distancia disponivel ate 5 m;
     *   5. calcula a desaceleracao necessaria para chegar
     *      ao ponto planejado;
     *
     * Depois disso, a desaceleracao e suavizada ao longo
     * da reacao para evitar uma freada brusca proxima ao V1.
     */

    /*
     * =========================================================
     * SCENARIO D - FRENAGEM PLANEJADA
     * =========================================================
     *
     * O planejamento e feito uma unica vez quando o alerta chega.
     *
     * Objetivo:
     *
     *     velocidade inicial -> 0 m/s
     *     distancia disponivel -> plannedTargetDistance
     *
     * Depois disso a velocidade alvo e atualizada apenas
     * pela desaceleracao planejada.
     *
     * Nao existe uma segunda fase de frenagem de emergencia.
     */

    /*
     * =========================================================
     * SCENARIO D - CONTROLE DINAMICO PELA DISTANCIA RELATIVA
     * =========================================================
     *
     * O calculo e renovado a cada 0,1 s utilizando:
     *
     *   - velocidade atual do V2;
     *   - velocidade atual do V1;
     *   - distancia atual entre os veiculos;
     *   - distancia alvo de 5 m.
     *
     * Dessa forma, o V1 pode continuar se movendo depois do alerta
     * sem invalidar o planejamento inicial.
     */

    /*
     * =========================================================
     * SCENARIO D - CONTROLE DINAMICO
     * =========================================================
     *
     * A desaceleracao e calculada continuamente em funcao de:
     *
     *   - velocidade atual do V2;
     *   - velocidade atual do V1;
     *   - distancia atual entre os veiculos;
     *   - distancia alvo de 5 m.
     *
     * O maxReactionDecel e somente um limite de seguranca.
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
         * Se a distancia alvo foi atingida, fazemos uma
         * reducao progressiva da velocidade em vez de mandar
         * imediatamente velocidade zero.
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
         * Se V2 ja esta mais lento que o lider, nao ha
         * necessidade de continuar freando.
         */
        if (currentSpeed <= leaderSpeed) {

            plannedReactionDecel = 0.0;
            calculatedReactionDecel = 0.0;

            return currentSpeed;
        }

        /*
         * =====================================================
         * DESACELERACAO NECESSARIA
         * =====================================================
         *
         * Queremos que V2 chegue a aproximadamente a velocidade
         * do lider quando a distancia chegar a 5 m.
         *
         * vf^2 = vi^2 - 2*a*d
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
         * A desaceleracao normal e dinamica.
         *
         * maxReactionDecel nao define a frenagem:
         * apenas impede que o algoritmo peca uma frenagem
         * acima do limite de seguranca.
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
         * de velocidade para o proximo intervalo.
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
     * O veiculo que recebe o alerta inicia a reacao.
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

        // Reinicia o plano do Scenario D para este alerta.
        plannedReactionInitialized = false;
        plannedReactionDecel = 0.0;
        plannedTargetDistance = 5.0;

        reducedSpeedReached = false;
        reactionActive = true;
        haveLastReactionPosition = true;
        lastReactionPosition = curPosition;

        /*
        * Primeira decisao de velocidade baseada na distancia
        * ate o veiculo a frente.
        */
        double initialReactionSpeed =
            calculateSafeReactionSpeed(speedAtAlert);

        /*
         * Primeira aplicacao do Scenario D.
         *
         * Tambem respeita o limite de desaceleracao por
         * intervalo, sem transformar esse limite em uma
         * desaceleracao fixa.
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
     * Diagnostico temporario apos a reacao V2X.
     * Executa somente para o V2 e no maximo uma vez por segundo.
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