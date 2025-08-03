package dev.UNYblade.ethereal_pathing.PathingGeneration.HybridPathing;

import dev.UNYblade.ethereal_pathing.Config.FieldConstants;
import dev.UNYblade.ethereal_pathing.Config.PathingConstants;
import dev.UNYblade.ethereal_pathing.PathingUtility.Waypoint;
import dev.UNYblade.ethereal_pathing.RobotUtility.Pose2d;

public class ClampedCubicSegment implements CurveSegment {
    private final Waypoint start, end;

    public ClampedCubicSegment(Waypoint start, Waypoint end) {
        this.start = start;
        this.end = end;

        // Optional: validate waypoints
        validateWaypoint(start);
        validateWaypoint(end);
    }

    @Override
    public Pose2d getPose(double t) {
        double x = cubicInterpolate(start.getX(), start.getDx(), end.getX(), end.getDx(), t);
        double y = cubicInterpolate(start.getY(), start.getDy(), end.getY(), end.getDy(), t);
        return new Pose2d(x, y, getHeading(t));
    }

    @Override
    public double getHeading(double t) {
        double dx = cubicDerivative(start.getX(), start.getDx(), end.getX(), end.getDx(), t);
        double dy = cubicDerivative(start.getY(), start.getDy(), end.getY(), end.getDy(), t);
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
        double dx = cubicDerivative(start.getX(), start.getDx(), end.getX(), end.getDx(), t);
        double dy = cubicDerivative(start.getY(), start.getDy(), end.getY(), end.getDy(), t);

        double heading = Math.atan2(dy, dx);
        double magnitude = Math.sqrt(dx * dx + dy * dy);

        return magnitude == 0
                ? new Pose2d(0, 0, heading)
                : new Pose2d(dx / magnitude, dy / magnitude, heading);
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

    private void validateWaypoint(Waypoint wp) {
        if (wp.getX() < 0 || wp.getX() > FieldConstants.FIELD_WIDTH ||
                wp.getY() < 0 || wp.getY() > FieldConstants.FIELD_HEIGHT) {
            System.out.println("Warning: Waypoint out of field bounds");
        }
    }
}