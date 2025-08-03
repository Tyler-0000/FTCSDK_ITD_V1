package dev.UNYblade.ethereal_pathing.PathingUtility;

import dev.UNYblade.ethereal_pathing.Config.PIDConstants;
import dev.UNYblade.ethereal_pathing.Config.PathingConstants;

public class PIDController {
    private double kP, kI, kD;
    private double integral, previousError;
    private double outputMin = Double.NEGATIVE_INFINITY;
    private double outputMax = Double.POSITIVE_INFINITY;
    private double integralLimit = Double.POSITIVE_INFINITY;

    public PIDController(double kP, double kI, double kD) {
        setGains(kP, kI, kD);
    }

    // Optional: default constructor using PIDConstants
    public PIDController() {
        setGains(PIDConstants.X_KP, PIDConstants.X_KI, PIDConstants.X_KD);
        setIntegralLimit(PathingConstants.DEFAULT_PATH_DURATION); // Example use
    }

    public void setGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setOutputLimits(double min, double max) {
        this.outputMin = min;
        this.outputMax = max;
    }

    public void setIntegralLimit(double limit) {
        this.integralLimit = limit;
    }

    public void reset() {
        integral = 0;
        previousError = 0;
    }

    public double calculate(double error) {
        integral += error;
        integral = clamp(integral, -integralLimit, integralLimit);

        double derivative = error - previousError;
        previousError = error;

        double output = kP * error + kI * integral + kD * derivative;
        return clamp(output, outputMin, outputMax);
    }

    public double calculate(double setpoint, double current) {
        return calculate(setpoint - current);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}