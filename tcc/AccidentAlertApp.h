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

    int alertsSent = 0;
    int alertsReceived = 0;

    omnetpp::simtime_t totalAlertDelay = 0;
    omnetpp::simtime_t minAlertDelay = -1;
    omnetpp::simtime_t maxAlertDelay = -1;

    // Metricas da reacao ao alerta
    omnetpp::simtime_t alertReceiveTime = -1;
    omnetpp::simtime_t timeToReducedSpeed = -1;

    double speedAtAlert = -1;
    double reactionTargetSpeed = -1;
    double minSpeedAfterAlert = -1;
    double reactionDistance = 0;

    // Scenario D: frenagem planejada
    bool plannedReactionInitialized = false;
    double plannedReactionDecel = 0.0;
    double plannedTargetDistance = 5.0;


    // Parametros da reacao V2X baseada em distancia
    std::string reactionStrategy = "current";
    double reactionSpeedFactor = 0.5;
    double safeDistance = 10.0;

    // Metricas adicionais
    double calculatedReactionDecel = 0.0;
    double currentLeaderDistance = -1.0;
    double progressiveAppliedDecel = 0.0;

    omnetpp::simtime_t reactionDuration = 5;
    omnetpp::simtime_t reactionMonitorInterval = 0.1;

    veins::Coord lastReactionPosition;

    // Funcao auxiliar do controlador de reacao
    double calculateSafeReactionSpeed(double currentSpeed);

    // Diagnostico temporario do comportamento apos a reacao
    omnetpp::simtime_t nextDiagnosticTime = 0;
};

}

#endif