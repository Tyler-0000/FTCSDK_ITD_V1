package org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing;

import org.firstinspires.ftc.teamcode.Pathing.Config.FieldConstants;
import org.firstinspires.ftc.teamcode.Pathing.Config.PathingConstants;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.PathingVelocity;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Waypoint;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Vector2d;

public class QuinticHermiteSegment implements CurveSegment {
    private final Waypoint start, end;

    public QuinticHermiteSegment(Waypoint start, Waypoint end) {
        this.start = start;
        this.end = end;
        validateWaypoint(start);
        validateWaypoint(end);
    }

    @Override
    public Pose2d getPose(double t) {
        double forward = quinticInterpolate(start.getX(), start.getDx(), start.getDdx(),
                end.getX(), end.getDx(), end.getDdx(), t);
        double strafe = quinticInterpolate(start.getY(), start.getDy(), start.getDdy(),
                end.getY(), end.getDy(), end.getDdy(), t);
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
        double df = quinticDerivative(start.getX(), start.getDx(), start.getDdx(),
                end.getX(), end.getDx(), end.getDdx(), t);
        double ds = quinticDerivative(start.getY(), start.getDy(), start.getDdy(),
                end.getY(), end.getDy(), end.getDdy(), t);
        return new Vector2d(df, ds).normalize();
    }

    @Override
    public PathingVelocity getVelocity(double t, double magnitude) {
        Vector2d tangent = getTangent(t);
        double headingRate = computeHeadingRate(t);
        return new PathingVelocity(
                tangent.x * magnitude, // forward
                tangent.y * magnitude, // strafe
                headingRate * magnitude
        );
    }

    @Override
    public double getCurvature(double t) {
        double df = quinticDerivative(start.getX(), start.getDx(), start.getDdx(),
                end.getX(), end.getDx(), end.getDdx(), t);
        double ds = quinticDerivative(start.getY(), start.getDy(), start.getDdy(),
                end.getY(), end.getDy(), end.getDdy(), t);
        double ddf = quinticSecondDerivative(start.getX(), start.getDx(), start.getDdx(),
                end.getX(), end.getDx(), end.getDdx(), t);
        double dds = quinticSecondDerivative(start.getY(), start.getDy(), start.getDdy(),
                end.getY(), end.getDy(), end.getDdy(), t);

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
        double df = quinticDerivative(start.getX(), start.getDx(), start.getDdx(),
                end.getX(), end.getDx(), end.getDdx(), t);
        double ds = quinticDerivative(start.getY(), start.getDy(), start.getDdy(),
                end.getY(), end.getDy(), end.getDdy(), t);
        double ddf = quinticSecondDerivative(start.getX(), start.getDx(), start.getDdx(),
                end.getX(), end.getDx(), end.getDdx(), t);
        double dds = quinticSecondDerivative(start.getY(), start.getDy(), start.getDdy(),
                end.getY(), end.getDy(), end.getDdy(), t);
        double denom = df * df + ds * ds;
        return denom == 0 ? 0 : (df * dds - ds * ddf) / denom;
    }

    private double quinticInterpolate(double p0, double v0, double a0,
                                      double p1, double v1, double a1, double t) {
        double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
        return p0 + v0 * t + 0.5 * a0 * t2
                + (-10 * p0 + 10 * p1 - 6 * v0 - 4 * v1 - 1.5 * a0 + 0.5 * a1) * t3
                + (15 * p0 - 15 * p1 + 8 * v0 + 7 * v1 + 1.5 * a0 - a1) * t4
                + (-6 * p0 + 6 * p1 - 3 * v0 - 3 * v1 - 0.5 * a0 + 0.5 * a1) * t5;
    }

    private double quinticDerivative(double p0, double v0, double a0,
                                     double p1, double v1, double a1, double t) {
        double t2 = t * t, t3 = t2 * t, t4 = t3 * t;
        return v0 + a0 * t
                + 3 * (-10 * p0 + 10 * p1 - 6 * v0 - 4 * v1 - 1.5 * a0 + 0.5 * a1) * t2
                + 4 * (15 * p0 - 15 * p1 + 8 * v0 + 7 * v1 + 1.5 * a0 - a1) * t3
                + 5 * (-6 * p0 + 6 * p1 - 3 * v0 - 3 * v1 - 0.5 * a0 + 0.5 * a1) * t4;
    }

    private double quinticSecondDerivative(double p0, double v0, double a0,
                                           double p1, double v1, double a1, double t) {
        double t1 = t, t2 = t * t, t3 = t2 * t;
        return a0
                + 6 * (-10 * p0 + 10 * p1 - 6 * v0 - 4 * v1 - 1.5 * a0 + 0.5 * a1) * t1
                + 12 * (15 * p0 - 15 * p1 + 8 * v0 + 7 * v1 + 1.5 * a0 - a1) * t2
                + 20 * (-6 * p0 + 6 * p1 - 3 * v0 - 3 * v1 - 0.5 * a0 + 0.5 * a1) * t3;
    }

    private void validateWaypoint(Waypoint wp) {
        if (wp.getX() < 0 || wp.getX() > FieldConstants.FIELD_WIDTH ||
                wp.getY() < 0 || wp.getY() > FieldConstants.FIELD_HEIGHT) {
            System.out.println("Warning: Waypoint out of field bounds");
        }
    }
}