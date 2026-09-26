#ifndef __TCC_ACCIDENTALERTAPP_H_
#define __TCC_ACCIDENTALERTAPP_H_

#include <veins/modules/application/ieee80211p/DemoBaseApplLayer.h>
#include <string>

namespace tcc {

class AccidentAlertApp : public veins::DemoBaseApplLayer
{
public:
    virtual ~AccidentAlertApp() override;
    
protected:
    virtual void initialize(int stage) override;
    virtual void handleSelfMsg(omnetpp::cMessage* msg) override;
    virtual void onWSM(veins::BaseFrame1609_4* wsm) override;
    virtual void handlePositionUpdate(omnetpp::cObject* obj) override;
    virtual void finish() override;

private:
    omnetpp::cMessage* accidentMsg = nullptr;
    omnetpp::cMessage* reactionMonitorMsg = nullptr;

    bool accidentTriggered = false;
    bool enableV2X = true;

    bool reactionActive = false;
    bool reducedSpeedReached = false;
    bool haveLastReactionPosition = false;

    bool rerouteTriggered = false;          // Para desvio
    simtime_t lowSpeedStart = -1;           // Para fallback

    int alertsSent = 0;
    int alertsReceived = 0;

    omnetpp::simtime_t totalAlertDelay = 0;
    omnetpp::simtime_t minAlertDelay = -1;
    omnetpp::simtime_t maxAlertDelay = -1;

    omnetpp::simtime_t alertReceiveTime = -1;
    omnetpp::simtime_t timeToReducedSpeed = -1;

    double speedAtAlert = -1;
    double reactionTargetSpeed = -1;
    double minSpeedAfterAlert = -1;
    double reactionDistance = 0;

    bool plannedReactionInitialized = false;
    double plannedReactionDecel = 0.0;
    double plannedTargetDistance = 5.0;

    std::string reactionStrategy = "current";
    double reactionSpeedFactor = 0.5;
    double safeDistance = 10.0;

    // SPI calculado em relação à velocidade de fluxo livre da via atual.
    // Não usamos histórico da via, porque o histórico pode ficar preso em
    // congestionamento e mascarar o estado real da estrada.
    double freeFlowSpeed = 0.0;
    double congestionSpiThreshold = 50.0;

    double calculatedReactionDecel = 0.0;
    double currentLeaderDistance = -1.0;
    double progressiveAppliedDecel = 0.0;

    omnetpp::simtime_t reactionDuration = 5;
    omnetpp::simtime_t reactionMonitorInterval = 0.1;

    veins::Coord lastReactionPosition;

    // Novo: para envio periódico de alertas de congestionamento
    simtime_t lastCongestionAlertTime = -1;

    simtime_t effectiveDetourEntryTime = -1;
    simtime_t commonRoadEntryTime = -1;

    double calculateSafeReactionSpeed(double currentSpeed);

    omnetpp::simtime_t nextDiagnosticTime = 0;
};

}

#endif
