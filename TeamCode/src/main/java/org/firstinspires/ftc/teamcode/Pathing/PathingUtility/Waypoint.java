package org.firstinspires.ftc.teamcode.Pathing.PathingUtility;

import org.firstinspires.ftc.teamcode.Pathing.Config.FieldConstants;

public class Waypoint {
    private final double x;
    private final double y;
    private final double heading; // in radians

    private final double dx;
    private final double dy;
    private final double ddx;
    private final double ddy;

    public Waypoint(double x, double y, double heading) {
        this(x, y, heading, Math.cos(heading), Math.sin(heading), 0, 0);
    }

    public Waypoint(double x, double y, double heading, double dx, double dy, double ddx, double ddy) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.dx = dx;
        this.dy = dy;
        this.ddx = ddx;
        this.ddy = ddy;

        validateBounds();
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }

    public double getDx() { return dx; }
    public double getDy() { return dy; }

    public double getDdx() { return ddx; }
    public double getDdy() { return ddy; }

    public Waypoint mirrorY() {
        return new Waypoint(x, -y, -heading, dx, -dy, ddx, -ddy);
    }

    public Waypoint offset(double dx, double dy) {
        return new Waypoint(x + dx, y + dy, heading, this.dx, this.dy, this.ddx, this.ddy);
    }

    public static Waypoint fromDegrees(double x, double y, double headingDegrees) {
        double headingRadians = Math.toRadians(headingDegrees);
        return new Waypoint(x, y, headingRadians);
    }

    private void validateBounds() {
        if (x < 0 || x > FieldConstants.FIELD_WIDTH || y < 0 || y > FieldConstants.FIELD_HEIGHT) {
            System.out.println("Warning: Waypoint out of field bounds");
        }
    }

    @Override
    public String toString() {
        return String.format("Waypoint(x=%.2f, y=%.2f, heading=%.2f rad)", x, y, heading);
    }
}