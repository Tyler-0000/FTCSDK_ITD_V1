package org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing;

import org.firstinspires.ftc.teamcode.Pathing.Config.FieldConstants;
import org.firstinspires.ftc.teamcode.Pathing.Config.PathingConstants;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.PathingVelocity;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Vector2d;

/**
 * Represents a cubic Bézier segment defined by four control points.
 * Assumes:
 *   - x = forward
 *   - y = strafe
 */
public class BezierSegment implements CurveSegment {
    private final Pose2d p0, p1, p2, p3;

    public BezierSegment(Pose2d p0, Pose2d p1, Pose2d p2, Pose2d p3) {
        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
        validatePose(p0);
        validatePose(p1);
        validatePose(p2);
        validatePose(p3);
    }

    @Override
    public Pose2d getPose(double t) {
        double forward = bezier(p0.x, p1.x, p2.x, p3.x, t);
        double strafe = bezier(p0.y, p1.y, p2.y, p3.y, t);
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
        double df = bezierDerivative(p0.x, p1.x, p2.x, p3.x, t); // forward
        double ds = bezierDerivative(p0.y, p1.y, p2.y, p3.y, t); // strafe
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
        double df = bezierDerivative(p0.x, p1.x, p2.x, p3.x, t);
        double ds = bezierDerivative(p0.y, p1.y, p2.y, p3.y, t);
        double ddf = bezierSecondDerivative(p0.x, p1.x, p2.x, p3.x, t);
        double dds = bezierSecondDerivative(p0.y, p1.y, p2.y, p3.y, t);

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
        double df = bezierDerivative(p0.x, p1.x, p2.x, p3.x, t);
        double ds = bezierDerivative(p0.y, p1.y, p2.y, p3.y, t);
        double ddf = bezierSecondDerivative(p0.x, p1.x, p2.x, p3.x, t);
        double dds = bezierSecondDerivative(p0.y, p1.y, p2.y, p3.y, t);
        double denom = df * df + ds * ds;
        return denom == 0 ? 0 : (df * dds - ds * ddf) / denom;
    }

    private double bezier(double a, double b, double c, double d, double t) {
        double u = 1 - t;
        return u * u * u * a + 3 * u * u * t * b + 3 * u * t * t * c + t * t * t * d;
    }

    private double bezierDerivative(double a, double b, double c, double d, double t) {
        double u = 1 - t;
        return 3 * u * u * (b - a) + 6 * u * t * (c - b) + 3 * t * t * (d - c);
    }

    private double bezierSecondDerivative(double a, double b, double c, double d, double t) {
        double u = 1 - t;
        return 6 * u * (c - 2 * b + a) + 6 * t * (d - 2 * c + b);
    }

    private void validatePose(Pose2d pose) {
        if (pose.x < 0 || pose.x > FieldConstants.FIELD_WIDTH ||
                pose.y < 0 || pose.y > FieldConstants.FIELD_HEIGHT) {
            System.out.println("Warning: Bézier control point out of field bounds");
        }
    }
}