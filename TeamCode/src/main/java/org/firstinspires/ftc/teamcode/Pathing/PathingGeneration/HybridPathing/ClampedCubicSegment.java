package org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing;

import org.firstinspires.ftc.teamcode.Pathing.Config.FieldConstants;
import org.firstinspires.ftc.teamcode.Pathing.Config.PathingConstants;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.PathingVelocity;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Waypoint;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Vector2d;

/**
 * Represents a clamped cubic segment between two waypoints.
 * Assumes x = forward, y = strafe.
 * Uses Hermite interpolation with velocity clamping.
 */
public class ClampedCubicSegment implements CurveSegment {
    private final Waypoint start, end;

    public ClampedCubicSegment(Waypoint start, Waypoint end) {
        this.start = start;
        this.end = end;
        validateWaypoint(start);
        validateWaypoint(end);
    }

    @Override
    public Pose2d getPose(double t) {
        double forward = cubicInterpolate(start.getX(), start.getDx(), end.getX(), end.getDx(), t);
        double strafe = cubicInterpolate(start.getY(), start.getDy(), end.getY(), end.getDy(), t);
        return new Pose2d(forward, strafe, getHeading(t));
    }

    @Override
    public double getHeading(double t) {
        Vector2d tangent = getTangent(t);
        return Math.atan2(tangent.y, tangent.x); // strafe over forward
    }

    @Override
    public double getLength() {
        double length = 0;
        Pose2d prev = getPose(0);
        for (double t = PathingConstants.SAMPLE_RESOLUTION; t <= 1.0; t += PathingConstants.SAMPLE_RESOLUTION) {
            Pose2d curr = getPose(t);
            length += curr.distanceTo(prev);
            prev = curr;
        }
        return length;
    }

    @Override
    public Vector2d getTangent(double t) {
        double df = cubicDerivative(start.getX(), start.getDx(), end.getX(), end.getDx(), t); // forward
        double ds = cubicDerivative(start.getY(), start.getDy(), end.getY(), end.getDy(), t); // strafe
        return new Vector2d(df, ds).normalize();
    }

    @Override
    public PathingVelocity getVelocity(double t, double magnitude) {
        Vector2d tangent = getTangent(t);
        double headingRate = computeHeadingRate(t);
        return new PathingVelocity(
                tangent.x * magnitude, // forward velocity
                tangent.y * magnitude, // strafe velocity
                headingRate * magnitude
        );
    }

    @Override
    public double getCurvature(double t) {
        double df = cubicDerivative(start.getX(), start.getDx(), end.getX(), end.getDx(), t);
        double ds = cubicDerivative(start.getY(), start.getDy(), end.getY(), end.getDy(), t);
        double ddf = cubicSecondDerivative(start.getDx(), end.getDx(), t);
        double dds = cubicSecondDerivative(start.getDy(), end.getDy(), t);

        double numerator = df * dds - ds * ddf;
        double denominator = Math.pow(df * df + ds * ds, 1.5);
        return denominator == 0 ? 0 : numerator / denominator;
    }

    public double getArcLengthTo(double tTarget) {
        double length = 0;
        Pose2d prev = getPose(0);
        for (double t = PathingConstants.SAMPLE_RESOLUTION; t <= tTarget; t += PathingConstants.SAMPLE_RESOLUTION) {
            Pose2d curr = getPose(t);
            length += curr.distanceTo(prev);
            prev = curr;
        }
        return length;
    }

    public Pose2d getMidpoint() {
        return getPose(0.5);
    }

    private double computeHeadingRate(double t) {
        double df = cubicDerivative(start.getX(), start.getDx(), end.getX(), end.getDx(), t);
        double ds = cubicDerivative(start.getY(), start.getDy(), end.getY(), end.getDy(), t);
        double ddf = cubicSecondDerivative(start.getDx(), end.getDx(), t);
        double dds = cubicSecondDerivative(start.getDy(), end.getDy(), t);
        double denominator = df * df + ds * ds;
        return denominator == 0 ? 0 : (df * dds - ds * ddf) / denominator;
    }

    private double cubicInterpolate(double p0, double m0, double p1, double m1, double t) {
        double t2 = t * t;
        double t3 = t2 * t;
        return (2 * t3 - 3 * t2 + 1) * p0
                + (t3 - 2 * t2 + t) * m0
                + (-2 * t3 + 3 * t2) * p1
                + (t3 - t2) * m1;
    }

    private double cubicDerivative(double p0, double m0, double p1, double m1, double t) {
        double t2 = t * t;
        return (6 * t2 - 6 * t) * p0
                + (3 * t2 - 4 * t + 1) * m0
                + (-6 * t2 + 6 * t) * p1
                + (3 * t2 - 2 * t) * m1;
    }

    private double cubicSecondDerivative(double m0, double m1, double t) {
        return 6 * (1 - t) * m0 + 6 * t * m1;
    }

    private void validateWaypoint(Waypoint wp) {
        if (wp.getX() < 0 || wp.getX() > FieldConstants.FIELD_WIDTH ||
                wp.getY() < 0 || wp.getY() > FieldConstants.FIELD_HEIGHT) {
            System.out.println("Warning: Waypoint out of field bounds");
        }
    }
}