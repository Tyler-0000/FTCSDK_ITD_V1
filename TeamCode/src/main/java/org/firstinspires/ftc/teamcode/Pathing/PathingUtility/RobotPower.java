package org.firstinspires.ftc.teamcode.Pathing.PathingUtility;

import org.firstinspires.ftc.teamcode.Pathing.Config.DriveConstants;
import org.firstinspires.ftc.teamcode.Pathing.Config.PathingConstants;

public class RobotPower {
    public double forward;
    public double strafe;
    public double turn;

    public RobotPower(double forward, double strafe, double turn) {
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
    }

    // --- Basic Arithmetic ---
    public RobotPower plus(RobotPower other) {
        return new RobotPower(
                this.forward + other.forward,
                this.strafe + other.strafe,
                this.turn + other.turn
        );
    }

    public RobotPower minus(RobotPower other) {
        return new RobotPower(
                this.forward - other.forward,
                this.strafe - other.strafe,
                this.turn - other.turn
        );
    }

    public RobotPower times(double factor) {
        return new RobotPower(forward * factor, strafe * factor, turn * factor);
    }

    public RobotPower scale(double factor) {
        return times(factor); // alias
    }

    public RobotPower invert() {
        return times(-1);
    }

    // --- Magnitude and Normalization ---
    public double getMagnitude() {
        return Math.sqrt(forward * forward + strafe * strafe + turn * turn);
    }

    public RobotPower normalize() {
        double mag = getMagnitude();
        return mag == 0 ? RobotPower.zero() : times(1.0 / mag);
    }

    public boolean isZero() {
        return Math.abs(forward) < 1e-3 && Math.abs(strafe) < 1e-3 && Math.abs(turn) < 1e-3;
    }

    public boolean isBelowThreshold() {
        return getMagnitude() < PathingConstants.MIN_VELOCITY;
    }

    // --- Clamping ---
    public RobotPower clampToLimits() {
        double max = DriveConstants.MAX_VELOCITY;
        return new RobotPower(
                clamp(forward, -max, max),
                clamp(strafe, -max, max),
                clamp(turn, -PathingConstants.MAX_ROTATION_VELOCITY, PathingConstants.MAX_ROTATION_VELOCITY)
        );
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // --- Advanced Vector Math ---
    public double dot(RobotPower other) {
        return this.forward * other.forward + this.strafe * other.strafe + this.turn * other.turn;
    }

    public RobotPower projectOnto(RobotPower other) {
        RobotPower unit = other.normalize();
        double magnitude = this.dot(unit);
        return unit.times(magnitude);
    }

    public double angleTo(RobotPower other) {
        double dot = this.dot(other);
        double magProduct = this.getMagnitude() * other.getMagnitude();
        if (magProduct == 0) return 0;
        return Math.acos(clamp(dot / magProduct, -1.0, 1.0)); // radians
    }

    // --- Conversion and Utility ---
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