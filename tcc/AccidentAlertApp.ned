package tcc;

simple AccidentAlertApp extends org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer
{
    parameters:
        bool enableV2X = default(true);
        string reactionStrategy = default("current");
        double reactionSpeedFactor = default(0.5);
        double safeDistance @unit("m") = default(10m);
        double reactionDuration @unit("s") = default(5s);
        double reactionMonitorInterval @unit("s") = default(0.1s);
        @class(tcc::AccidentAlertApp);
}
