package org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing;

import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.PathingVelocity;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Vector2d;

/**
 * Represents a segment of a continuous path.
 * Supports sampling pose, heading, velocity, curvature, and tangent direction.
 *
 * Coordinate convention:
 *   - x = forward
 *   - y = strafe
 */
public interface CurveSegment {

    /**
     * Returns the interpolated pose at parameter t ∈ [0, 1].
     * Includes position and heading.
     */
    Pose2d getPose(double t);

    /**
     * Returns the heading (in radians) at parameter t ∈ [0, 1].
     * Typically derived from the tangent vector.
     */
    double getHeading(double t);

    /**
     * Returns the total arc length of the segment.
     * Used for time scaling and progress tracking.
     */
    double getLength();

    /**
     * Returns the normalized tangent vector at parameter t.
     * Represents direction of motion (not magnitude).
     */
    Vector2d getTangent(double t);

    /**
     * Returns the velocity vector at parameter t, scaled by magnitude.
     * Includes forward, strafe, and rotational components.
     */
    PathingVelocity getVelocity(double t, double magnitude);

    /**
     * Returns the curvature at parameter t.
     * Curvature = rate of heading change per unit distance.
     */
    double getCurvature(double t);
}