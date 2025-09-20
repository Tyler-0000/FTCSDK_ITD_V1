package org.firstinspires.ftc.teamcode.Pathing.PathingGeneration;

import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing.BezierSegment;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing.BezierCurveGenerator;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing.ClampedCubicSegment;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing.ClampedCubicGenerator;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing.LinearBezierSegment;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing.QuinticHermiteSegment;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing.QuinticHermiteGenerator;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing.CurveSegment;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Waypoint;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;


import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class HybridPathBuilder {

    /**
     * Builds a curve-based path from a list of (x, y, heading) poses.
     * Automatically selects the best curve generator based on spacing and curvature.
     */
    public static List<CurveSegment> buildCurvePath(List<Pose2d> poses) {
        List<Waypoint> waypoints = inferWaypoints(poses);

        if (poses.size() < 3) {
            return BezierCurveGenerator.fromWaypoints(poses); // fallback for short paths
        }

        double avgCurvature = estimateAverageCurvature(poses);
        if (avgCurvature > 0.05) {
            return QuinticHermiteGenerator.fromWaypoints(waypoints);
        } else {
            return ClampedCubicGenerator.fromWaypoints(waypoints);
        }
    }

    /**
     * Builds a line-based path from a list of (x, y, heading) poses.
     * Uses Clamped Cubic segments for smooth transitions.
     */
    public enum SegmentType {
        CLAMPED_CUBIC,
        QUINTIC_HERMITE,
        BEZIER
    }

    public static List<CurveSegment> buildLinearPath(List<Pose2d> poses, SegmentType type) {
        List<Waypoint> waypoints = inferWaypoints(poses);
        List<CurveSegment> segments = new ArrayList<>();

        for (int i = 0; i < waypoints.size() - 1; i++) {
            Waypoint start = waypoints.get(i);
            Waypoint end = waypoints.get(i + 1);

            switch (type) {
                case CLAMPED_CUBIC:
                    segments.add(new ClampedCubicSegment(start, end));
                    break;
                case QUINTIC_HERMITE:
                    segments.add(new QuinticHermiteSegment(start, end));
                    break;
                case BEZIER:
                    segments.add(new LinearBezierSegment(start.toPose2d(), end.toPose2d()));
                    break;
            }
        }

        return segments;
    }

    /**
     * Converts Pose2d into Waypoint with inferred velocity and acceleration.
     * Uses heading to estimate directional derivatives.
     */
    private static List<Waypoint> inferWaypoints(List<Pose2d> poses) {
        List<Waypoint> waypoints = new ArrayList<>();

        for (int i = 0; i < poses.size(); i++) {
            Pose2d pose = poses.get(i);

            double dx = 0, dy = 0;

            // Estimate velocity from adjacent poses
            if (i > 0 && i < poses.size() - 1) {
                Pose2d prev = poses.get(i - 1);
                Pose2d next = poses.get(i + 1);

                dx = (next.x - prev.x) / 2.0;
                dy = (next.y - prev.y) / 2.0;
            } else if (i < poses.size() - 1) {
                Pose2d next = poses.get(i + 1);
                dx = next.x - pose.x;
                dy = next.y - pose.y;
            } else if (i > 0) {
                Pose2d prev = poses.get(i - 1);
                dx = pose.x - prev.x;
                dy = pose.y - prev.y;
            }

            // Estimate acceleration (second derivative)
            double ddx = 0, ddy = 0;
            if (i > 0 && i < poses.size() - 1) {
                Pose2d prev = poses.get(i - 1);
                Pose2d next = poses.get(i + 1);
                ddx = (next.x - 2 * pose.x + prev.x);
                ddy = (next.y - 2 * pose.y + prev.y);
            }

            waypoints.add(new Waypoint(
                    pose.x, pose.y, pose.heading,
                    dx, dy, ddx, ddy
            ));
        }

        return waypoints;
    }

    public static void logWaypoints(List<Pose2d> poses, Telemetry telemetry) {
        List<Waypoint> waypoints = inferWaypoints(poses);
        for (int i = 0; i < waypoints.size(); i++) {
            Waypoint wp = waypoints.get(i);
            telemetry.addData("Waypoint " + i,
                    String.format(Locale.ENGLISH, "x=%.2f y=%.2f h=%.2f dx=%.2f dy=%.2f ddx=%.2f ddy=%.2f",
                            wp.getX(), wp.getY(), wp.getHeading(), wp.getDx(), wp.getDy(), wp.getDdx(), wp.getDdy()));
        }
    }

    /**
     * Estimates average curvature across a list of poses.
     * Used to select curve generator.
     */
    private static double estimateAverageCurvature(List<Pose2d> poses) {
        double totalCurvature = 0;
        int count = 0;

        for (int i = 1; i < poses.size() - 1; i++) {
            Pose2d a = poses.get(i - 1);
            Pose2d b = poses.get(i);
            Pose2d c = poses.get(i + 1);

            double angleChange = Math.abs(normalizeAngle(c.heading - a.heading));
            double distance = b.distanceTo(a) + b.distanceTo(c);
            if (distance > 0) {
                totalCurvature += angleChange / distance;
                count++;
            }
        }

        return count == 0 ? 0 : totalCurvature / count;
    }

    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}