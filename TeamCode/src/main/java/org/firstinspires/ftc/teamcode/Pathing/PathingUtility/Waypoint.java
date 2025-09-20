package org.firstinspires.ftc.teamcode.Pathing.PathingUtility;

import org.firstinspires.ftc.teamcode.Pathing.Config.FieldConstants;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

/**
 * Represents a waypoint in robot-centric coordinates.
 * Includes position, heading, and optional velocity/acceleration derivatives.
 * Assumes:
 *   - x = forward
 *   - y = strafe
 *   - heading = orientation in radians
 */
public class Waypoint {
    private final double x;       // forward position
    private final double y;       // strafe position
    private final double heading; // orientation in radians

    private final double dx;      // forward velocity component
    private final double dy;      // strafe velocity component
    private final double ddx;     // forward acceleration component
    private final double ddy;     // strafe acceleration component

    /** Constructs a waypoint with heading-derived velocity. */
    public Waypoint(double x, double y, double heading) {
        this(x, y, heading, Math.cos(heading), Math.sin(heading), 0, 0);
    }

    /** Constructs a waypoint with full derivative specification. */
    public Waypoint(double x, double y, double heading,
                    double dx, double dy,
                    double ddx, double ddy) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.dx = dx;
        this.dy = dy;
        this.ddx = ddx;
        this.ddy = ddy;

        validateBounds();
    }

    // Getters for position and heading
    public double getX() { return x; }           // forward
    public double getY() { return y; }           // strafe
    public double getHeading() { return heading; }

    // Getters for velocity components
    public double getDx() { return dx; }         // forward velocity
    public double getDy() { return dy; }         // strafe velocity

    // Getters for acceleration components
    public double getDdx() { return ddx; }       // forward acceleration
    public double getDdy() { return ddy; }       // strafe acceleration

    /** Returns a mirrored waypoint across the Y-axis (strafe flip). */
    public Waypoint mirrorY() {
        return new Waypoint(x, -y, -heading, dx, -dy, ddx, -ddy);
    }

    /** Returns a new waypoint offset by (dx, dy) in field space. */
    public Waypoint offset(double dx, double dy) {
        return new Waypoint(x + dx, y + dy, heading, this.dx, this.dy, this.ddx, this.ddy);
    }

    /** Constructs a waypoint using heading in degrees. */
    public static Waypoint fromDegrees(double x, double y, double headingDegrees) {
        double headingRadians = Math.toRadians(headingDegrees);
        return new Waypoint(x, y, headingRadians);
    }

    /** Validates that the waypoint is within field bounds. */
    private void validateBounds() {
        if (x < 0 || x > FieldConstants.FIELD_WIDTH ||
                y < 0 || y > FieldConstants.FIELD_HEIGHT) {
            System.out.println("Warning: Waypoint out of field bounds");
        }
    }

    @Override
    public String toString() {
        return String.format("Waypoint(x=%.2f, y=%.2f, heading=%.2f rad)", x, y, heading);
    }

    /** Converts this waypoint to a Pose2d (position + heading only). */
    public Pose2d toPose2d() {
        return new Pose2d(x, y, heading);
    }
}