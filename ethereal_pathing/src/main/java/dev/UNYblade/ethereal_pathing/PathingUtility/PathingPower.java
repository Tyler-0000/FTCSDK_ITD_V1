package dev.UNYblade.ethereal_pathing.PathingUtility;

import dev.UNYblade.ethereal_pathing.Config.DriveConstants;
import dev.UNYblade.ethereal_pathing.Config.PathingConstants;

public class PathingPower {
    public double vertical;
    public double horizontal;
    public double rotation;

    public PathingPower(double vertical, double horizontal, double rotation) {
        this.vertical = vertical;
        this.horizontal = horizontal;
        this.rotation = rotation;
    }

    public PathingPower scale(double factor) {
        return new PathingPower(vertical * factor, horizontal * factor, rotation * factor);
    }

    public PathingPower invert() {
        return new PathingPower(-vertical, -horizontal, -rotation);
    }

    public double getMagnitude() {
        return Math.sqrt(vertical * vertical + horizontal * horizontal + rotation * rotation);
    }

    public PathingPower normalize() {
        double mag = getMagnitude();
        return mag == 0 ? new PathingPower(0, 0, 0) : scale(1.0 / mag);
    }

    public PathingPower clampToMaxVelocity() {
        double max = DriveConstants.MAX_VELOCITY;
        return new PathingPower(
                clamp(vertical, -max, max),
                clamp(horizontal, -max, max),
                clamp(rotation, -PathingConstants.MAX_ROTATION_VELOCITY, PathingConstants.MAX_ROTATION_VELOCITY)
        );
    }

    public boolean isBelowThreshold() {
        return getMagnitude() < PathingConstants.MIN_VELOCITY;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public String toString() {
        return String.format("PathingPower(v=%.3f, h=%.3f, r=%.3f)", vertical, horizontal, rotation);
    }
}