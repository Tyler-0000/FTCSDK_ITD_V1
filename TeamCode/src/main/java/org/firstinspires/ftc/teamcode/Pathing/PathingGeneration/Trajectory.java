package org.firstinspires.ftc.teamcode.Pathing.PathingGeneration;

import org.firstinspires.ftc.teamcode.Pathing.Config.FieldConstants;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing.CurveSegment;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.PathingVelocity;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a composed trajectory made of multiple curve segments.
 * Supports pose sampling, velocity lookup, curvature analysis, and reversal.
 * Assumes:
 *   - x = forward
 *   - y = strafe
 */
public class Trajectory {

    private final List<CurveSegment> segments;
    private final List<Double> segmentLengths;
    private final double totalLength;
    private final double maxVelocity;
    private final boolean reversed;

    private List<PathingVelocity> overrideVelocities = null;

    public Trajectory(List<CurveSegment> segments, double maxVelocity, boolean reversed) {
        this.segments = segments;
        this.maxVelocity = maxVelocity;
        this.reversed = reversed;
        this.segmentLengths = new ArrayList<>();

        double lengthSum = 0;
        for (CurveSegment segment : segments) {
            double len = segment.getLength();
            segmentLengths.add(len);
            lengthSum += len;

            Pose2d endPose = segment.getPose(1.0);
            if (endPose.x < 0 || endPose.x > FieldConstants.FIELD_WIDTH ||
                    endPose.y < 0 || endPose.y > FieldConstants.FIELD_HEIGHT) {
                System.out.println("Warning: Segment endpoint out of field bounds");
            }
        }
        this.totalLength = lengthSum;
    }

    public Pose2d getPoseAtDistance(double distance) {
        if (distance >= totalLength) {
            Pose2d finalPose = segments.get(segments.size() - 1).getPose(1.0);
            return reversed ? reversePose(finalPose) : finalPose;
        }

        double accumulated = 0;
        for (int i = 0; i < segments.size(); i++) {
            double segLen = segmentLengths.get(i);
            if (accumulated + segLen >= distance) {
                double localT = (distance - accumulated) / segLen;
                Pose2d pose = segments.get(i).getPose(localT);
                return reversed ? reversePose(pose) : pose;
            }
            accumulated += segLen;
        }

        Pose2d finalPose = segments.get(segments.size() - 1).getPose(1.0);
        return reversed ? reversePose(finalPose) : finalPose;
    }

    public PathingVelocity getVelocityAtPose(Pose2d pose) {
        double distance = getDistanceAtPose(pose);
        return getVelocityAtDistance(distance);
    }

    public PathingVelocity getVelocityAtDistance(double distance) {
        if (overrideVelocities != null) {
            int index = (int) ((distance / totalLength) * overrideVelocities.size());
            return overrideVelocities.get(Math.min(index, overrideVelocities.size() - 1));
        }

        if (distance >= totalLength) return PathingVelocity.zero();

        double accumulated = 0;
        for (int i = 0; i < segments.size(); i++) {
            double segLen = segmentLengths.get(i);
            if (accumulated + segLen >= distance) {
                double localT = (distance - accumulated) / segLen;
                PathingVelocity velocity = segments.get(i).getVelocity(localT, maxVelocity);
                return reversed ? velocity.negated() : velocity;
            }
            accumulated += segLen;
        }

        return PathingVelocity.zero();
    }

    public Pose2d getClosestPose(Pose2d currentPose) {
        Pose2d closest = segments.get(0).getPose(0);
        double minDist = Double.MAX_VALUE;
        double resolution = totalLength / 100.0;

        for (double d = 0; d <= totalLength; d += resolution) {
            Pose2d sample = getPoseAtDistance(d);
            double dist = sample.distanceTo(currentPose);
            if (dist < minDist) {
                minDist = dist;
                closest = sample;
            }
        }
        return closest;
    }

    public Pose2d getLookaheadPose(Pose2d currentPose, double offsetDistance) {
        double currentDistance = getDistanceAtPose(currentPose);
        double lookaheadDistance = Math.min(currentDistance + offsetDistance, totalLength);
        return getPoseAtDistance(lookaheadDistance);
    }

    public boolean isNearEnd(Pose2d currentPose) {
        return currentPose.distanceTo(getPoseAtDistance(totalLength)) < 3.0;
    }

    public boolean isFinished(Pose2d currentPose) {
        return currentPose.distanceTo(getPoseAtDistance(totalLength)) < 1.0;
    }

    public double getDistanceAtPose(Pose2d targetPose) {
        double bestDistance = 0;
        double minDist = Double.MAX_VALUE;
        double resolution = totalLength / 100.0;

        for (double d = 0; d <= totalLength; d += resolution) {
            Pose2d sample = getPoseAtDistance(d);
            double dist = sample.distanceTo(targetPose);
            if (dist < minDist) {
                minDist = dist;
                bestDistance = d;
            }
        }
        return bestDistance;
    }

    public double getCurvatureAtDistance(double distance) {
        double accumulated = 0;
        for (int i = 0; i < segments.size(); i++) {
            double segLen = segmentLengths.get(i);
            if (accumulated + segLen >= distance) {
                double localT = (distance - accumulated) / segLen;
                return segments.get(i).getCurvature(localT);
            }
            accumulated += segLen;
        }
        return 0;
    }

    public List<Pose2d> getSamples(int count) {
        List<Pose2d> samples = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            double d = (i / (double)(count - 1)) * totalLength;
            samples.add(getPoseAtDistance(d));
        }
        return samples;
    }

    public List<Double> getHeadingList(int count) {
        List<Double> headings = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            double d = (i / (double)(count - 1)) * totalLength;
            headings.add(getPoseAtDistance(d).heading);
        }
        return headings;
    }

    public List<PathingVelocity> getVelocityList(int count) {
        List<PathingVelocity> velocities = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            double d = (i / (double)(count - 1)) * totalLength;
            velocities.add(getVelocityAtDistance(d));
        }
        return velocities;
    }

    public List<Double> getCurvatureList(int count) {
        List<Double> curvatures = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            double d = (i / (double)(count - 1)) * totalLength;
            curvatures.add(getCurvatureAtDistance(d));
        }
        return curvatures;
    }

    public double getTotalLength() {
        return totalLength;
    }

    public void setOverrideVelocities(List<PathingVelocity> velocities) {
        this.overrideVelocities = velocities;
    }

    public List<CurveSegment> getSegments() {
        return segments;
    }

    public int getSegmentIndexAtDistance(double distance) {
        double accumulated = 0;
        for (int i = 0; i < segments.size(); i++) {
            double segLen = segmentLengths.get(i);
            if (accumulated + segLen >= distance) {
                return i;
            }
            accumulated += segLen;
        }
        return segments.size() - 1;
    }

    private Pose2d reversePose(Pose2d pose) {
        return new Pose2d(pose.x, pose.y, normalizeAngle(pose.heading + Math.PI));
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public List<Pose2d> getWaypoints() {
        List<Pose2d> waypoints = new ArrayList<>();
        for (CurveSegment segment : segments) {
            waypoints.add(segment.getPose(0.0)); // Start of segment
        }
        // Add final endpoint
        waypoints.add(segments.get(segments.size() - 1).getPose(1.0));
        return waypoints;
    }
}