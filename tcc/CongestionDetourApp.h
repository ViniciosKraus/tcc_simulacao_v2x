#ifndef __TCC_CONGESTIONDETOURAPP_H_
#define __TCC_CONGESTIONDETOURAPP_H_

#include <veins/modules/application/ieee80211p/DemoBaseApplLayer.h>
#include <string>

namespace tcc {

class CongestionDetourApp : public veins::DemoBaseApplLayer
{
public:
    virtual ~CongestionDetourApp() override;
    
protected:
    virtual void initialize(int stage) override;
    virtual void handleSelfMsg(omnetpp::cMessage* msg) override;
    virtual void onWSM(veins::BaseFrame1609_4* wsm) override;
    virtual void handlePositionUpdate(omnetpp::cObject* obj) override;
    virtual void finish() override;

private:
    omnetpp::cMessage* congestionMonitorMsg = nullptr;

    bool enableV2X = true;
    std::string vehicleTypeId;

    bool rerouteTriggered = false;          // Para desvio

    double congestionSpiThreshold = 50.0;
    int congestionSpiPersistenceChecks = 10;
    int lowSpiConsecutiveChecks = 0;

    omnetpp::simtime_t congestionMonitorInterval = 0.1;

    simtime_t lastCongestionAlertTime = -1;

    simtime_t effectiveDetourEntryTime = -1;
    simtime_t commonRoadEntryTime = -1;
    int alertSequence = 0;
    int alertsSent = 0;
    int alertsReceived = 0;
    int alertLatencyCount = 0;
    double alertLatencySum = 0.0;
    bool alertTriggeredReroute = false;
};

}

#endif
