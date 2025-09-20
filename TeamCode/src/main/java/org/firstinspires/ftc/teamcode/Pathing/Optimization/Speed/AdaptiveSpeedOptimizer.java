package org.firstinspires.ftc.teamcode.Pathing.Optimization.Speed;

import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

public class AdaptiveSpeedOptimizer implements SpeedOptimizer {
    private final double maxVelocity;
    private final double voltageNominal;
    private final double accelerationBoostFactor;

    public AdaptiveSpeedOptimizer(double maxVelocity, double voltageNominal, double accelerationBoostFactor) {
        this.maxVelocity = maxVelocity;
        this.voltageNominal = voltageNominal;
        this.accelerationBoostFactor = accelerationBoostFactor;
    }

    @Override
    public RobotPower optimizeSpeed(RobotPower desiredPower, Pose2d currentPose, double batteryVoltage) {
        double voltageRatio = Math.min(1.0, batteryVoltage / voltageNominal);
        double velocityRatio = desiredPower.getMagnitude() / maxVelocity;

        // If we're below target velocity, boost output slightly
        double boost = (1.0 - velocityRatio) * accelerationBoostFactor * voltageRatio;
        double scale = 1.0 + boost;

        return desiredPower.times(scale).clampToLimits();
    }
}