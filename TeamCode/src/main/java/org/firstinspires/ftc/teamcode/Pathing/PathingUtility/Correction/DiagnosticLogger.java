package org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction;

import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;

public interface DiagnosticLogger {
    void log(Pose2d currentPose, Pose2d targetPose, RobotPower rawCorrection, RobotPower scaledCorrection, RobotPower finalPower);
}