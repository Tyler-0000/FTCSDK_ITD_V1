package org.firstinspires.ftc.teamcode.Pathing.PathingGeneration;

import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing.BezierSegment;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing.ClampedCubicSegment;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing.QuinticHermiteSegment;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing.CurveSegment;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Waypoint;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

import java.util.ArrayList;
import java.util.List;

/**
 * Composes a hybrid trajectory from multiple curve segments.
 * Supports Bézier, Clamped Cubic, and Quintic Hermite interpolation.
 * Assumes:
 *   - x = forward
 *   - y = strafe
 */
public class PathComposer {

    private final List<CurveSegment> segments = new ArrayList<>();
    private double maxVelocity = 40.0;
    private boolean reversed = false;

    /**
     * Sets the maximum velocity for the composed trajectory.
     * @param maxVelocity velocity in inches per second
     */
    public void setMaxVelocity(double maxVelocity) {
        this.maxVelocity = maxVelocity;
    }

    /**
     * Sets whether the trajectory should be followed in reverse.
     * @param reversed true if reversed, false if forward
     */
    public void setReversed(boolean reversed) {
        this.reversed = reversed;
    }

    /**
     * Adds a cubic Bézier segment using four control points.
     * @param p0 start point
     * @param p1 first control point
     * @param p2 second control point
     * @param p3 end point
     */
    public void addBezierSegment(Pose2d p0, Pose2d p1, Pose2d p2, Pose2d p3) {
        segments.add(new BezierSegment(p0, p1, p2, p3));
    }

    /**
     * Adds a clamped cubic segment using two waypoints.
     * Waypoints include position and directional derivatives.
     * @param start starting waypoint
     * @param end ending waypoint
     */
    public void addClampedCubicSegment(Waypoint start, Waypoint end) {
        segments.add(new ClampedCubicSegment(start, end));
    }

    /**
     * Adds a quintic Hermite segment using two waypoints.
     * Waypoints include position, velocity, and acceleration.
     * @param start starting waypoint
     * @param end ending waypoint
     */
    public void addQuinticHermiteSegment(Waypoint start, Waypoint end) {
        segments.add(new QuinticHermiteSegment(start, end));
    }

    /**
     * Builds a trajectory from the composed segments.
     * @return a Trajectory object with velocity and direction settings
     */
    public Trajectory buildTrajectory() {
        return new Trajectory(segments, maxVelocity, reversed);
    }

    /**
     * Returns the list of curve segments in the current composition.
     */
    public List<CurveSegment> getSegments() {
        return segments;
    }

    /**
     * Clears all segments from the current composition.
     */
    public void clear() {
        segments.clear();
    }
}