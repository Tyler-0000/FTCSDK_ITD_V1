package org.firstinspires.ftc.teamcode.Pathing.PathingGeneration;

import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing.BezierSegment;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing.CurveSegment;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.PathingVelocity;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

import java.util.ArrayList;
import java.util.List;

/**
 * Utility class for constructing trajectories from segments or raw points.
 * Assumes:
 *   - x = forward
 *   - y = strafe
 */
public class TrajectoryFactory {

    /**
     * Builds a trajectory directly from a list of curve segments.
     *
     * @param segments     List of CurveSegment objects
     * @param maxVelocity  Maximum velocity for the trajectory
     * @param reversed     Whether the trajectory should be followed in reverse
     * @return             A composed Trajectory object
     */
    public static Trajectory fromSegments(List<CurveSegment> segments, double maxVelocity, boolean reversed) {
        return new Trajectory(segments, maxVelocity, reversed);
    }

    /**
     * Builds a trajectory from a list of Pose2d points using Bézier segments.
     * Control points are auto-generated using midpoint heuristics.
     *
     * @param points       List of Pose2d waypoints
     * @param maxVelocity  Maximum velocity for the trajectory
     * @param reversed     Whether the trajectory should be followed in reverse
     * @return             A composed Trajectory object
     */
    public static Trajectory fromPoints(List<Pose2d> points, double maxVelocity, boolean reversed) {
        List<CurveSegment> segments = new ArrayList<>();

        for (int i = 0; i < points.size() - 1; i++) {
            Pose2d p0 = points.get(i);
            Pose2d p3 = points.get(i + 1);

            // Midpoint-based control point generation
            double cx = (p0.x + p3.x) / 2;
            double cy = (p0.y + p3.y) / 2;

            Pose2d p1 = new Pose2d((p0.x + cx) / 2, (p0.y + cy) / 2, 0);
            Pose2d p2 = new Pose2d((p3.x + cx) / 2, (p3.y + cy) / 2, 0);

            segments.add(new BezierSegment(p0, p1, p2, p3));
        }

        return new Trajectory(segments, maxVelocity, reversed);
    }

    /** Samples curvature values along the trajectory. */
    public static List<Double> sampleCurvature(Trajectory traj, int count) {
        return traj.getCurvatureList(count);
    }

    /** Samples heading angles along the trajectory. */
    public static List<Double> sampleHeadings(Trajectory traj, int count) {
        return traj.getHeadingList(count);
    }

    /** Samples velocity vectors along the trajectory. */
    public static List<PathingVelocity> sampleVelocities(Trajectory traj, int count) {
        return traj.getVelocityList(count);
    }

    /** Samples poses (position + heading) along the trajectory. */
    public static List<Pose2d> samplePoses(Trajectory traj, int count) {
        return traj.getSamples(count);
    }
}