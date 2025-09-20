package org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction;

import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;

public interface CorrectionBlender {
    RobotPower blend(RobotPower base, RobotPower correction);
}