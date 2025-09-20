package org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation;

import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.DiagnosticLogger;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryLogger implements DiagnosticLogger {
    private final Telemetry telemetry;

    public TelemetryLogger(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void log(Pose2d currentPose, Pose2d targetPose, RobotPower raw, RobotPower scaled, RobotPower finalPower) {
        telemetry.addData("Target Pose", targetPose);
        telemetry.addData("Current Pose", currentPose);
        telemetry.addData("Raw Correction", raw);
        telemetry.addData("Scaled Correction", scaled);
        telemetry.addData("Final Power", finalPower);
    }
}