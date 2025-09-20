package org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing;

import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing.CurveSegment;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing.QuinticHermiteSegment;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Waypoint;

import java.util.ArrayList;
import java.util.List;

public class QuinticHermiteGenerator {

    /**
     * Generates QuinticHermiteSegments from a list of waypoints.
     * Each waypoint includes position, velocity (dx, dy), and acceleration (ddx, ddy).
     * Assumes x = forward, y = strafe.
     */
    public static List<CurveSegment> fromWaypoints(List<Waypoint> waypoints) {
        List<CurveSegment> segments = new ArrayList<>();

        for (int i = 0; i < waypoints.size() - 1; i++) {
            Waypoint start = waypoints.get(i);
            Waypoint end = waypoints.get(i + 1);
            segments.add(new QuinticHermiteSegment(start, end));
        }

        return segments;
    }
}