package org.firstinspires.ftc.teamcode.Pathing.PathingUtility;

import org.firstinspires.ftc.teamcode.Pathing.Config.DriveConstants;
import org.firstinspires.ftc.teamcode.Pathing.Config.PathingConstants;

public class PathingVelocity {
    public double xVelocity;
    public double yVelocity;
    public double rotationVelocity;

    public PathingVelocity(double xVelocity, double yVelocity, double rotationVelocity) {
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        this.rotationVelocity = rotationVelocity;
    }

    public PathingVelocity scale(double factor) {
        return new PathingVelocity(
                xVelocity * factor,
                yVelocity * factor,
                rotationVelocity * factor
        );
    }

    public PathingVelocity add(PathingVelocity other) {
        return new PathingVelocity(
                this.xVelocity + other.xVelocity,
                this.yVelocity + other.yVelocity,
                this.rotationVelocity + other.rotationVelocity
        );
    }

    public double getMagnitude() {
        return Math.sqrt(xVelocity * xVelocity + yVelocity * yVelocity + rotationVelocity * rotationVelocity);
    }

    public PathingVelocity normalize() {
        double mag = getMagnitude();
        return mag == 0 ? new PathingVelocity(0, 0, 0) : scale(1.0 / mag);
    }

    public PathingVelocity clampToLimits() {
        double max = DriveConstants.MAX_VELOCITY;
        return new PathingVelocity(
                clamp(xVelocity, -max, max),
                clamp(yVelocity, -max, max),
                clamp(rotationVelocity, -PathingConstants.MAX_ROTATION_VELOCITY, PathingConstants.MAX_ROTATION_VELOCITY)
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
        return String.format("PathingVelocity(x=%.3f, y=%.3f, r=%.3f)", xVelocity, yVelocity, rotationVelocity);
    }
}