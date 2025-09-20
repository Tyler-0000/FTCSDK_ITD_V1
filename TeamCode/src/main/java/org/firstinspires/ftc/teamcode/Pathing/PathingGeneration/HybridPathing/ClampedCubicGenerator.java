package org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing;

import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Waypoint;
import java.util.ArrayList;
import java.util.List;

/**
 * Generates a list of ClampedCubicSegments from a sequence of waypoints.
 * Each segment interpolates position and directional derivatives (dx, dy).
 * Assumes:
 *   - x = forward
 *   - y = strafe
 */
public class ClampedCubicGenerator {

    /**
     * Converts a list of waypoints into clamped cubic segments.
     * Each segment uses Hermite interpolation with velocity clamping.
     *
     * @param waypoints List of waypoints with position and directional derivatives
     * @return List of CurveSegments representing the full path
     */
    public static List<CurveSegment> fromWaypoints(List<Waypoint> waypoints) {
        List<CurveSegment> segments = new ArrayList<>();

        for (int i = 0; i < waypoints.size() - 1; i++) {
            Waypoint start = waypoints.get(i);
            Waypoint end = waypoints.get(i + 1);
            segments.add(new ClampedCubicSegment(start, end));
        }

        return segments;
    }
}