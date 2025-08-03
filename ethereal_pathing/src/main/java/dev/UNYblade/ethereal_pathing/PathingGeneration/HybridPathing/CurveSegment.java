package dev.UNYblade.ethereal_pathing.PathingGeneration.HybridPathing;

import dev.UNYblade.ethereal_pathing.RobotUtility.Pose2d;

public interface CurveSegment {
    // Returns position at a given t (0.0 to 1.0)
    Pose2d getPose(double t);

    // Returns heading at a given t
    double getHeading(double t);

    // Returns total length of the segment
    double getLength();
    public abstract Pose2d getTangent(double t);

}
