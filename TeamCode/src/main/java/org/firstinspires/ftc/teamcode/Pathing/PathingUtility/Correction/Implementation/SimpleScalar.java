package org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation;

import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.CorrectionScaler;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;

public class SimpleScalar implements CorrectionScaler {
    private double forwardScale = 0.8;
    private double strafeScale = 0.8;
    private double turnScale = 1.0;

    public SimpleScalar(double forwardScale, double strafeScale, double turnScale) {
        this.forwardScale = forwardScale;
        this.strafeScale = strafeScale;
        this.turnScale = turnScale;
    }

    @Override
    public RobotPower scale(RobotPower correction, Pose2d currentPose, double dt) {
        return new RobotPower(
                correction.forward * forwardScale,
                correction.strafe * strafeScale,
                correction.turn * turnScale
        );
    }
}