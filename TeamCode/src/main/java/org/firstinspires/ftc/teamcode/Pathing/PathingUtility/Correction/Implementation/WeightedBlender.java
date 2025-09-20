package org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation;

import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.CorrectionBlender;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;

public class WeightedBlender implements CorrectionBlender {
    private double baseWeight = 1.0;
    private double correctionWeight = 1.0;

    public WeightedBlender(double baseWeight, double correctionWeight) {
        this.baseWeight = baseWeight;
        this.correctionWeight = correctionWeight;
    }

    @Override
    public RobotPower blend(RobotPower base, RobotPower correction) {
        return new RobotPower(
                base.forward * baseWeight + correction.forward * correctionWeight,
                base.strafe * baseWeight + correction.strafe * correctionWeight,
                base.turn * baseWeight + correction.turn * correctionWeight
        );
    }
}
