package org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing;

import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.PathingVelocity;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Vector2d;

public class LinearBezierSegment implements CurveSegment {
    private final Pose2d p0, p1;

    public LinearBezierSegment(Pose2d p0, Pose2d p1) {
        this.p0 = p0;
        this.p1 = p1;
    }

    @Override
    public Pose2d getPose(double t) {
        double x = (1 - t) * p0.x + t * p1.x;
        double y = (1 - t) * p0.y + t * p1.y;
        double heading = (1 - t) * p0.heading + t * p1.heading;
        return new Pose2d(x, y, heading);
    }

    @Override
    public double getHeading(double t) {
        return getPose(t).heading;
    }

    @Override
    public double getLength() {
        return p0.distanceTo(p1);
    }

    @Override
    public Vector2d getTangent(double t) {
        return new Vector2d(p1.x - p0.x, p1.y - p0.y).normalize();
    }

    @Override
    public PathingVelocity getVelocity(double t, double magnitude) {
        Vector2d tangent = getTangent(t);
        return new PathingVelocity(tangent.x * magnitude, tangent.y * magnitude, 0);
    }

    @Override
    public double getCurvature(double t) {
        return 0; // straight line
    }
}