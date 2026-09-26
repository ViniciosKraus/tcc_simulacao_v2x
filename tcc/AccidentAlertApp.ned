package tcc;

@namespace(tcc);

simple AccidentAlertApp extends veins.modules.application.ieee80211p.DemoBaseApplLayer
{
    parameters:
        bool enableV2X = default(true);
        string reactionStrategy = default("current");
        double reactionSpeedFactor = default(0.5);
        double safeDistance @unit("m") = default(10m);
        double reactionDuration @unit("s") = default(5s);
        double reactionMonitorInterval @unit("s") = default(0.1s);
        double freeFlowSpeed @unit("mps") = default(0mps);
        double congestionSpiThreshold = default(50.0);
        @class(AccidentAlertApp);
}
