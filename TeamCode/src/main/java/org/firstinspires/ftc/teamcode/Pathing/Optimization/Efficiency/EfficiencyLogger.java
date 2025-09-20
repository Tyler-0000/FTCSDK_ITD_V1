package org.firstinspires.ftc.teamcode.Pathing.Optimization.Efficiency;

import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class EfficiencyLogger {
    private final Telemetry telemetry;

    public EfficiencyLogger(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void log(RobotPower original, RobotPower optimized) {
        telemetry.addData("Original Power", original);
        telemetry.addData("Optimized Power", optimized);
        telemetry.addData("Efficiency Ratio", optimized.getMagnitude() / original.getMagnitude());
    }
}