package org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing;

import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.PathingVelocity;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Vector2d;

/**
 * Represents a straight-line segment between two poses.
 * Assumes x = forward, y = strafe.
 */
public class LinearSegment implements CurveSegment {

    private final Pose2d start;
    private final Pose2d end;
    private final double length;
    private final Vector2d direction;

    public LinearSegment(Pose2d start, Pose2d end) {
        this.start = start;
        this.end = end;
        this.length = start.distanceTo(end);
        this.direction = new Vector2d(end.x - start.x, end.y - start.y).normalize(); // forward/strafe direction
    }

    @Override
    public Pose2d getPose(double t) {
        t = clamp(t, 0, 1);
        double forward = start.x + (end.x - start.x) * t;
        double strafe = start.y + (end.y - start.y) * t;
        double heading = getHeading(t);
        return new Pose2d(forward, strafe, heading);
    }

    @Override
    public double getHeading(double t) {
        return start.heading + (end.heading - start.heading) * clamp(t, 0, 1);
    }

    @Override
    public double getLength() {
        return length;
    }

    @Override
    public Vector2d getTangent(double t) {
        return direction; // constant direction for linear segment
    }

    @Override
    public PathingVelocity getVelocity(double t, double magnitude) {
        Vector2d tangent = getTangent(t);
        double headingRate = end.heading - start.heading; // constant rotation rate
        return new PathingVelocity(
                tangent.x * magnitude, // forward velocity
                tangent.y * magnitude, // strafe velocity
                headingRate            // rotational velocity
        );
    }

    @Override
    public double getCurvature(double t) {
        return 0.0; // straight line = zero curvature
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}