package org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction;

import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;

public interface CorrectionScaler {
    RobotPower scale(RobotPower correction, Pose2d currentPose, double dt);
}