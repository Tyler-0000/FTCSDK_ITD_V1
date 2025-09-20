package org.firstinspires.ftc.teamcode.Pathing.Optimization.Efficiency;

import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

public class AdvancedEfficiencyModel implements EfficiencyModel {
    private final double frictionCoefficient;
    private final double dragFactor;
    private final double weightFactor;
    private final double voltageNominal;

    public AdvancedEfficiencyModel(double frictionCoefficient, double dragFactor, double weightFactor, double voltageNominal) {
        this.frictionCoefficient = frictionCoefficient;
        this.dragFactor = dragFactor;
        this.weightFactor = weightFactor;
        this.voltageNominal = voltageNominal;
    }

    @Override
    public RobotPower optimize(RobotPower inputPower, Pose2d currentPose, double batteryVoltage) {
        double velocity = inputPower.getMagnitude();

        // Estimate resistive force
        double resistance = frictionCoefficient + dragFactor * velocity + weightFactor;

        // Estimate available efficiency based on voltage
        double voltageEfficiency = Math.min(1.0, batteryVoltage / voltageNominal);

        // Compute axis-wise scaling to preserve direction but reduce strain
        double forwardScale = computeAxisEfficiency(inputPower.forward, resistance, voltageEfficiency);
        double strafeScale = computeAxisEfficiency(inputPower.strafe, resistance, voltageEfficiency);
        double turnScale = computeAxisEfficiency(inputPower.turn, resistance, voltageEfficiency);

        return new RobotPower(
                inputPower.forward * forwardScale,
                inputPower.strafe * strafeScale,
                inputPower.turn * turnScale
        );
    }

    private double computeAxisEfficiency(double component, double resistance, double voltageEfficiency) {
        if (Math.abs(component) < 1e-4) return 0;
        double effort = Math.abs(component);
        double efficiencyPenalty = resistance * effort;
        double scale = voltageEfficiency * (1.0 - efficiencyPenalty);
        return Math.max(0.5, scale); // Prevent underpowering
    }
}