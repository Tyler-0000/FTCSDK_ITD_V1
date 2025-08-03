package dev.UNYblade.ethereal_pathing.PathingGeneration.HybridPathing;

import dev.UNYblade.ethereal_pathing.Config.FieldConstants;
import dev.UNYblade.ethereal_pathing.Config.PathingConstants;
import dev.UNYblade.ethereal_pathing.PathingUtility.Waypoint;
import dev.UNYblade.ethereal_pathing.RobotUtility.Pose2d;

public class QuinticHermiteSegment implements CurveSegment {
    private final Waypoint start, end;

    public QuinticHermiteSegment(Waypoint start, Waypoint end) {
        this.start = start;
        this.end = end;

        // Optional: validate waypoints
        validateWaypoint(start);
        validateWaypoint(end);
    }

    @Override
    public Pose2d getPose(double t) {
        double x = quinticInterpolate(start.getX(), start.getDx(), start.getDdx(),
                end.getX(), end.getDx(), end.getDdx(), t);
        double y = quinticInterpolate(start.getY(), start.getDy(), start.getDdy(),
                end.getY(), end.getDy(), end.getDdy(), t);
        return new Pose2d(x, y, getHeading(t));
    }

    @Override
    public double getHeading(double t) {
        double dx = quinticDerivative(start.getX(), start.getDx(), start.getDdx(),
                end.getX(), end.getDx(), end.getDdx(), t);
        double dy = quinticDerivative(start.getY(), start.getDy(), start.getDdy(),
                end.getY(), end.getDy(), end.getDdy(), t);
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
        double dx = quinticDerivative(start.getX(), start.getDx(), start.getDdx(),
                end.getX(), end.getDx(), end.getDdx(), t);
        double dy = quinticDerivative(start.getY(), start.getDy(), start.getDdy(),
                end.getY(), end.getDy(), end.getDdy(), t);

        double heading = Math.atan2(dy, dx);
        double magnitude = Math.sqrt(dx * dx + dy * dy);

        return magnitude == 0
                ? new Pose2d(0, 0, heading)
                : new Pose2d(dx / magnitude, dy / magnitude, heading);
    }

    private double quinticInterpolate(double p0, double v0, double a0,
                                      double p1, double v1, double a1, double t) {
        double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
        return p0 + v0 * t + 0.5 * a0 * t2 +
                (-10 * p0 + 10 * p1 - 6 * v0 - 4 * v1 - 1.5 * a0 + 0.5 * a1) * t3 +
                (15 * p0 - 15 * p1 + 8 * v0 + 7 * v1 + 1.5 * a0 - a1) * t4 +
                (-6 * p0 + 6 * p1 - 3 * v0 - 3 * v1 - 0.5 * a0 + 0.5 * a1) * t5;
    }

    private double quinticDerivative(double p0, double v0, double a0,
                                     double p1, double v1, double a1, double t) {
        double t2 = t * t, t3 = t2 * t, t4 = t3 * t;
        return v0 + a0 * t +
                3 * (-10 * p0 + 10 * p1 - 6 * v0 - 4 * v1 - 1.5 * a0 + 0.5 * a1) * t2 +
                4 * (15 * p0 - 15 * p1 + 8 * v0 + 7 * v1 + 1.5 * a0 - a1) * t3 +
                5 * (-6 * p0 + 6 * p1 - 3 * v0 - 3 * v1 - 0.5 * a0 + 0.5 * a1) * t4;
    }

    private void validateWaypoint(Waypoint wp) {
        if (wp.getX() < 0 || wp.getX() > FieldConstants.FIELD_WIDTH ||
                wp.getY() < 0 || wp.getY() > FieldConstants.FIELD_HEIGHT) {
            System.out.println("Warning: Waypoint out of field bounds");
        }
    }
}