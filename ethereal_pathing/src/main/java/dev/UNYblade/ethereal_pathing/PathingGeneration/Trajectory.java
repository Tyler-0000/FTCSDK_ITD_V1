package dev.UNYblade.ethereal_pathing.PathingGeneration;

import java.util.ArrayList;
import java.util.List;

import dev.UNYblade.ethereal_pathing.Config.FieldConstants;
import dev.UNYblade.ethereal_pathing.Config.PathingConstants;
import dev.UNYblade.ethereal_pathing.PathingGeneration.HybridPathing.CurveSegment;
import dev.UNYblade.ethereal_pathing.PathingUtility.PathingVelocity;
import dev.UNYblade.ethereal_pathing.RobotUtility.Pose2d;

public class Trajectory {
    private final List<CurveSegment> segments;
    private final List<Double> segmentLengths;
    private final double totalLength;
    private final double maxVelocity;
    private final double acceleration;
    private final boolean reversed;

    private final double accelTime;
    private final double cruiseTime;
    private final double decelTime;
    private final double totalTime;

    public Trajectory(List<CurveSegment> segments, double maxVelocity, double acceleration, boolean reversed) {
        this.segments = segments;
        this.maxVelocity = maxVelocity;
        this.acceleration = acceleration;
        this.reversed = reversed;
        this.segmentLengths = new ArrayList<>();

        double lengthSum = 0;
        for (CurveSegment segment : segments) {
            double len = segment.getLength();
            segmentLengths.add(len);
            lengthSum += len;

            // Optional: validate segment endpoints
            Pose2d endPose = segment.getPose(1.0);
            if (endPose.getX() < 0 || endPose.getX() > FieldConstants.FIELD_WIDTH ||
                    endPose.getY() < 0 || endPose.getY() > FieldConstants.FIELD_HEIGHT) {
                System.out.println("Warning: Segment endpoint out of field bounds");
            }
        }
        this.totalLength = lengthSum;

        // Trapezoidal motion profile
        double accelDist = (maxVelocity * maxVelocity) / (2 * acceleration);
        if (2 * accelDist > totalLength) {
            accelTime = Math.sqrt(totalLength / acceleration);
            decelTime = accelTime;
            cruiseTime = 0;
        } else {
            accelTime = maxVelocity / acceleration;
            decelTime = accelTime;
            cruiseTime = (totalLength - 2 * accelDist) / maxVelocity;
        }

        totalTime = accelTime + cruiseTime + decelTime;
    }

    public Pose2d getPoseAt(double time) {
        double distance = getDistanceAt(time);

        if (distance >= totalLength) {
            Pose2d finalPose = segments.get(segments.size() - 1).getPose(1.0);
            return reversed ? new Pose2d(finalPose.getX(), finalPose.getY(), finalPose.getHeading() + Math.PI) : finalPose;
        }

        double accumulated = 0;
        for (int i = 0; i < segments.size(); i++) {
            double segLen = segmentLengths.get(i);
            if (accumulated + segLen >= distance) {
                double localT = (distance - accumulated) / segLen;
                Pose2d pose = segments.get(i).getPose(localT);
                return reversed ? new Pose2d(pose.getX(), pose.getY(), pose.getHeading() + Math.PI) : pose;
            }
            accumulated += segLen;
        }

        Pose2d finalPose = segments.get(segments.size() - 1).getPose(1.0);
        return reversed ? new Pose2d(finalPose.getX(), finalPose.getY(), finalPose.getHeading() + Math.PI) : finalPose;
    }

    private double getVelocityAt(double time) {
        if (time < accelTime) {
            return acceleration * time;
        } else if (time < accelTime + cruiseTime) {
            return maxVelocity;
        } else if (time < totalTime) {
            return maxVelocity - acceleration * (time - accelTime - cruiseTime);
        } else {
            return 0;
        }
    }

    private double getDistanceAt(double time) {
        double dist = 0;
        double dt = PathingConstants.SAMPLE_RESOLUTION;
        for (double t = 0; t < time; t += dt) {
            dist += getVelocityAt(t) * dt;
        }
        return dist;
    }

    public double getTotalTime() {
        return totalTime;
    }

    public double getTotalLength() {
        return totalLength;
    }

    public List<Pose2d> getSamples(int count) {
        List<Pose2d> samples = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            double t = (i / (double)(count - 1)) * totalTime;
            samples.add(getPoseAt(t));
        }
        return samples;
    }

    public PathingVelocity getVelocity(double time) {
        double velocityMagnitude = getVelocityAt(time);
        double distance = getDistanceAt(time);

        if (distance >= totalLength) {
            return new PathingVelocity(0, 0, 0);
        }

        double accumulated = 0;
        for (int i = 0; i < segments.size(); i++) {
            double segLen = segmentLengths.get(i);
            if (accumulated + segLen >= distance) {
                double localT = (distance - accumulated) / segLen;
                Pose2d pose = segments.get(i).getPose(localT);
                Pose2d tangent = segments.get(i).getTangent(localT);

                double xVel = tangent.getX() * velocityMagnitude;
                double yVel = tangent.getY() * velocityMagnitude;
                double rotVel = tangent.getHeading() * velocityMagnitude;

                return reversed
                        ? new PathingVelocity(-xVel, -yVel, -rotVel)
                        : new PathingVelocity(xVel, yVel, rotVel);
            }
            accumulated += segLen;
        }

        return new PathingVelocity(0, 0, 0);
    }
}