package org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing;

import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

import java.util.ArrayList;
import java.util.List;

/**
 * Utility class for generating cubic Bézier curve segments.
 * Assumes:
 *   - x = forward
 *   - y = strafe
 */
public class BezierCurveGenerator {

    /**
     * Generates a list of BezierSegments from a list of control points.
     * Each group of 4 consecutive points defines one cubic Bézier curve.
     *
     * @param controlPoints List of Pose2d control points
     * @return List of CurveSegments representing Bézier curves
     */
    public static List<CurveSegment> fromControlPoints(List<Pose2d> controlPoints) {
        List<CurveSegment> segments = new ArrayList<>();
        for (int i = 0; i + 3 < controlPoints.size(); i += 3) {
            Pose2d p0 = controlPoints.get(i);
            Pose2d p1 = controlPoints.get(i + 1);
            Pose2d p2 = controlPoints.get(i + 2);
            Pose2d p3 = controlPoints.get(i + 3);
            segments.add(new BezierSegment(p0, p1, p2, p3));
        }
        return segments;
    }

    /**
     * Generates Bézier segments from a list of waypoints using midpoint-based control heuristics.
     * This method auto-generates control points for smooth transitions.
     *
     * @param waypoints List of Pose2d waypoints
     * @return List of CurveSegments representing Bézier curves
     */
    public static List<CurveSegment> fromWaypoints(List<Pose2d> waypoints) {
        List<CurveSegment> segments = new ArrayList<>();
        for (int i = 0; i < waypoints.size() - 1; i++) {
            Pose2d p0 = waypoints.get(i);
            Pose2d p3 = waypoints.get(i + 1);

            // Midpoint between p0 and p3
            double cx = (p0.x + p3.x) / 2;
            double cy = (p0.y + p3.y) / 2;

            // Control points biased toward midpoint
            Pose2d p1 = new Pose2d((p0.x + cx) / 2, (p0.y + cy) / 2, 0);
            Pose2d p2 = new Pose2d((p3.x + cx) / 2, (p3.y + cy) / 2, 0);

            segments.add(new BezierSegment(p0, p1, p2, p3));
        }
        return segments;
    }
}