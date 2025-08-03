package dev.UNYblade.ethereal_pathing.PathingUtility;

import dev.UNYblade.ethereal_pathing.Config.DriveConstants;
import dev.UNYblade.ethereal_pathing.Config.PathingConstants;

public class RobotPower {
    public double forward;
    public double strafe;
    public double turn;

    public RobotPower(double forward, double strafe, double turn) {
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
    }

    public RobotPower scale(double factor) {
        return new RobotPower(forward * factor, strafe * factor, turn * factor);
    }

    public RobotPower invert() {
        return new RobotPower(-forward, -strafe, -turn);
    }

    public double getMagnitude() {
        return Math.sqrt(forward * forward + strafe * strafe + turn * turn);
    }

    public RobotPower normalize() {
        double mag = getMagnitude();
        return mag == 0 ? new RobotPower(0, 0, 0) : scale(1.0 / mag);
    }

    public RobotPower clampToLimits() {
        double max = DriveConstants.MAX_VELOCITY;
        return new RobotPower(
                clamp(forward, -max, max),
                clamp(strafe, -max, max),
                clamp(turn, -PathingConstants.MAX_ROTATION_VELOCITY, PathingConstants.MAX_ROTATION_VELOCITY)
        );
    }

    public boolean isBelowThreshold() {
        return getMagnitude() < PathingConstants.MIN_VELOCITY;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static RobotPower fromPathingPower(PathingPower power) {
        return new RobotPower(power.vertical, power.horizontal, power.rotation);
    }

    public static RobotPower zero() {
        return new RobotPower(0, 0, 0);
    }

    @Override
    public String toString() {
        return String.format("RobotPower(f=%.3f, s=%.3f, t=%.3f)", forward, strafe, turn);
    }
}