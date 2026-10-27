package tcc;

@namespace(tcc);

simple CongestionDetourApp extends veins.modules.application.ieee80211p.DemoBaseApplLayer
{
    parameters:
        bool enableV2X = default(true);
        bool useVehicleTypeForV2X = default(false);
        double congestionMonitorInterval @unit("s") = default(0.1s);
        double congestionSpiThreshold = default(50.0);
        int congestionSpiPersistenceChecks = default(10);
        @class(CongestionDetourApp);
}
