package dev.UNYblade.ethereal_pathing.PathingGeneration.HybridPathing;

import dev.UNYblade.ethereal_pathing.Config.FieldConstants;
import dev.UNYblade.ethereal_pathing.Config.PathingConstants;
import dev.UNYblade.ethereal_pathing.RobotUtility.Pose2d;

public class BezierSegment implements CurveSegment {
    private final Pose2d p0, p1, p2, p3;

    public BezierSegment(Pose2d p0, Pose2d p1, Pose2d p2, Pose2d p3) {
        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;

        // Optional: validate control points
        validatePose(p0);
        validatePose(p1);
        validatePose(p2);
        validatePose(p3);
    }

    @Override
    public Pose2d getPose(double t) {
        double x = bezier(p0.getX(), p1.getX(), p2.getX(), p3.getX(), t);
        double y = bezier(p0.getY(), p1.getY(), p2.getY(), p3.getY(), t);
        return new Pose2d(x, y, getHeading(t));
    }

    @Override
    public double getHeading(double t) {
        double dx = bezierDerivative(p0.getX(), p1.getX(), p2.getX(), p3.getX(), t);
        double dy = bezierDerivative(p0.getY(), p1.getY(), p2.getY(), p3.getY(), t);
        return Math.atan2(dy, dx);
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
    public Pose2d getTangent(double t) {
        double dx = bezierDerivative(p0.getX(), p1.getX(), p2.getX(), p3.getX(), t);
        double dy = bezierDerivative(p0.getY(), p1.getY(), p2.getY(), p3.getY(), t);

        double heading = Math.atan2(dy, dx);
        double magnitude = Math.sqrt(dx * dx + dy * dy);

        return magnitude == 0
                ? new Pose2d(0, 0, heading)
                : new Pose2d(dx / magnitude, dy / magnitude, heading);
    }

    private double bezier(double a, double b, double c, double d, double t) {
        double u = 1 - t;
        return u * u * u * a + 3 * u * u * t * b + 3 * u * t * t * c + t * t * t * d;
    }

    private double bezierDerivative(double a, double b, double c, double d, double t) {
        double u = 1 - t;
        return 3 * u * u * (b - a) + 6 * u * t * (c - b) + 3 * t * t * (d - c);
    }

    private void validatePose(Pose2d pose) {
        if (pose.getX() < 0 || pose.getX() > FieldConstants.FIELD_WIDTH ||
                pose.getY() < 0 || pose.getY() > FieldConstants.FIELD_HEIGHT) {
            System.out.println("Warning: Bézier control point out of field bounds");
        }
    }
}