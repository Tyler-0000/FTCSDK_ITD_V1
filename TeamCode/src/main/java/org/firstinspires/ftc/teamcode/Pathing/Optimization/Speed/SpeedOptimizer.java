package org.firstinspires.ftc.teamcode.Pathing.Optimization.Speed;

import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

public interface SpeedOptimizer {
    RobotPower optimizeSpeed(RobotPower desiredPower, Pose2d currentPose, double batteryVoltage);
}