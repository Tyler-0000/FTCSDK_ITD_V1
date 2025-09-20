package org.firstinspires.ftc.teamcode.Pathing.PathingUtility;

import org.firstinspires.ftc.teamcode.Pathing.Config.DriveConstants;
import org.firstinspires.ftc.teamcode.Pathing.Config.PathingConstants;

/**
 * Represents a velocity vector in robot-centric coordinates.
 * Assumes:
 *   - xVelocity = forward motion
 *   - yVelocity = strafe (lateral) motion
 *   - rotationVelocity = angular velocity (radians/sec)
 */
public class PathingVelocity {
    public double xVelocity;         // forward
    public double yVelocity;         // strafe
    public double rotationVelocity;  // angular

    public PathingVelocity(double xVelocity, double yVelocity, double rotationVelocity) {
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        this.rotationVelocity = rotationVelocity;
    }

    /** Returns a new velocity scaled by the given factor. */
    public PathingVelocity scale(double factor) {
        return new PathingVelocity(
                xVelocity * factor,
                yVelocity * factor,
                rotationVelocity * factor
        );
    }

    /** Returns a new velocity that is the sum of this and another. */
    public PathingVelocity add(PathingVelocity other) {
        return new PathingVelocity(
                this.xVelocity + other.xVelocity,
                this.yVelocity + other.yVelocity,
                this.rotationVelocity + other.rotationVelocity
        );
    }

    /** Returns the Euclidean magnitude of the velocity vector. */
    public double getMagnitude() {
        return Math.sqrt(
                xVelocity * xVelocity +
                        yVelocity * yVelocity +
                        rotationVelocity * rotationVelocity
        );
    }

    /** Returns a normalized velocity vector (unit magnitude). */
    public PathingVelocity normalize() {
        double mag = getMagnitude();
        return mag == 0 ? zero() : scale(1.0 / mag);
    }

    /** Returns a velocity clamped to configured max limits. */
    public PathingVelocity clampToLimits() {
        double max = DriveConstants.MAX_VELOCITY;
        return new PathingVelocity(
                clamp(xVelocity, -max, max),
                clamp(yVelocity, -max, max),
                clamp(rotationVelocity,
                        -PathingConstants.MAX_ROTATION_VELOCITY,
                        PathingConstants.MAX_ROTATION_VELOCITY)
        );
    }

    /** Returns true if the velocity magnitude is below the minimum threshold. */
    public boolean isBelowThreshold() {
        return getMagnitude() < PathingConstants.MIN_VELOCITY;
    }

    /** Returns a velocity with all components negated. */
    public PathingVelocity negated() {
        return new PathingVelocity(-xVelocity, -yVelocity, -rotationVelocity);
    }

    /** Returns a zero velocity vector. */
    public static PathingVelocity zero() {
        return new PathingVelocity(0, 0, 0);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public String toString() {
        return String.format("PathingVelocity(x=%.3f, y=%.3f, r=%.3f)",
                xVelocity, yVelocity, rotationVelocity);
    }
}