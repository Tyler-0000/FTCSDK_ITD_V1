package org.firstinspires.ftc.teamcode.Pathing.Optimization.Efficiency;

import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

public class BasicEfficiencyModel implements EfficiencyModel {
    private final double frictionCoefficient;
    private final double dragFactor;
    private final double weightFactor;

    public BasicEfficiencyModel(double frictionCoefficient, double dragFactor, double weightFactor) {
        this.frictionCoefficient = frictionCoefficient;
        this.dragFactor = dragFactor;
        this.weightFactor = weightFactor;
    }

    @Override
    public RobotPower optimize(RobotPower inputPower, Pose2d currentPose, double batteryVoltage) {
        double velocity = inputPower.getMagnitude();
        double resistance = frictionCoefficient + dragFactor * velocity + weightFactor;

        double efficiencyScale = batteryVoltage > 12.0 ? 1.0 : batteryVoltage / 12.0;
        double adjustedScale = Math.max(0.5, efficiencyScale * (1.0 - resistance));

        return inputPower.times(adjustedScale);
    }
}