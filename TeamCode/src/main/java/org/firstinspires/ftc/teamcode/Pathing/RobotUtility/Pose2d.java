package org.firstinspires.ftc.teamcode.Pathing.RobotUtility;

import org.firstinspires.ftc.teamcode.Pathing.Config.FieldConstants;

public class Pose2d {
    private final double x;
    private final double y;
    private final double heading; // in radians

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = normalizeAngle(heading);
        validateBounds();
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }

    public double distanceTo(Pose2d other) {
        double dx = other.x - this.x;
        double dy = other.y - this.y;
        return Math.hypot(dx, dy);
    }

    public Pose2d subtract(Pose2d other) {
        return new Pose2d(this.x - other.x, this.y - other.y, this.heading - other.heading);
    }

    public Pose2d interpolate(Pose2d other, double t) {
        double ix = this.x + (other.x - this.x) * t;
        double iy = this.y + (other.y - this.y) * t;
        double ih = this.heading + (other.heading - this.heading) * t;
        return new Pose2d(ix, iy, ih);
    }

    public Pose2d mirrorY() {
        return new Pose2d(x, -y, -heading);
    }

    public static Pose2d fromDegrees(double x, double y, double headingDegrees) {
        return new Pose2d(x, y, Math.toRadians(headingDegrees));
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private void validateBounds() {
        if (x < 0 || x > FieldConstants.FIELD_WIDTH || y < 0 || y > FieldConstants.FIELD_HEIGHT) {
            System.out.println("Warning: Pose2d out of field bounds");
        }
    }

    @Override
    public String toString() {
        return String.format("Pose2d(x=%.2f, y=%.2f, heading=%.2f rad)", x, y, heading);
    }
}