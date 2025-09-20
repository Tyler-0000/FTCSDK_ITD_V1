package org.firstinspires.ftc.teamcode.Pathing.Optimization.Efficiency;

import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

public interface EfficiencyModel {
    RobotPower optimize(RobotPower inputPower, Pose2d currentPose, double batteryVoltage);
}
