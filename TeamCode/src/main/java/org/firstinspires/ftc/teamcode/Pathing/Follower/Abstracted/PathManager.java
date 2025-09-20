package org.firstinspires.ftc.teamcode.Pathing.Follower.Abstracted;

import org.firstinspires.ftc.teamcode.Pathing.Follower.follower;
import org.firstinspires.ftc.teamcode.Pathing.Follower.pathOperator;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathBuilder;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.Trajectory;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.PathingVelocity;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Odometry;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;
import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.*;
import org.firstinspires.ftc.teamcode.Pathing.Optimization.Efficiency.EfficiencyModel;
import org.firstinspires.ftc.teamcode.Pathing.Optimization.Speed.SpeedOptimizer;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * High-level manager for path execution and diagnostics.
 * Assumes:
 *   - x = forward
 *   - y = strafe
 */
public class PathManager {

    private final pathOperator operator;;

    public PathManager(OdometryHardware hardware,
                       Telemetry telemetry,
                       CorrectionStrategy strategy,
                       CorrectionScaler scaler,
                       CorrectionBlender blender,
                       DiagnosticLogger logger,
                       EfficiencyModel efficiencyModel,
                       SpeedOptimizer speedOptimizer) {

        Odometry odometry = new Odometry(hardware); // Constructs odometry from hardware
        this.operator = new pathOperator(
                odometry,
                hardware,
                telemetry,
                strategy,
                scaler,
                blender,
                logger,
                efficiencyModel,
                speedOptimizer
        );
    }

    /** Adds a curve-based trajectory using Bézier segments. */
    public void addCurvePath(List<Pose2d> poses, double maxVelocity, boolean reversed) {
        Trajectory trajectory = new Trajectory(
                HybridPathBuilder.buildCurvePath(poses),
                maxVelocity,
                reversed
        );
        operator.addPath(trajectory);
    }

    /** Adds a linear trajectory using clamped cubic segments. */
    public void addLinearPath(List<Pose2d> poses, HybridPathBuilder.SegmentType type, double maxVelocity, boolean reversed) {
        Trajectory trajectory = new Trajectory(
                HybridPathBuilder.buildLinearPath(poses, type),
                maxVelocity,
                reversed
        );
        operator.addPath(trajectory);
    }

    /** Starts execution of the trajectory at the given index. */
    public void startPath(int index) {
        operator.startPath(index);
    }

    /** Starts the next trajectory in the queue, if available. */
    public void startNextPath() {
        int nextIndex = operator.getCurrentIndex() + 1;
        if (nextIndex < operator.getTotalPaths()) {
            operator.startPath(nextIndex);
        }
    }

    /** Updates the follower and applies control output to hardware. */
    public void update() {
        operator.updateAndDrive();
    }

    /** Returns true if the current trajectory is finished. */
    public boolean isFinished() {
        return operator.isCurrentPathFinished();
    }

    /** Returns progress along the current path (0.0 to 1.0). */
    public double getProgress() {
        Pose2d current = operator.getCurrentPose(); // x = forward, y = strafe
        return operator.getProgress(current);
    }

    /** Returns the current pose error in (forward, strafe, heading). */
    public Pose2d getPoseError() {
        return operator.getPoseError();
    }

    /** Returns true if the robot is deviating beyond threshold. */
    public boolean isDeviating() {
        return operator.isDeviating();
    }

    /** Clears all queued paths and resets state. */
    public void clearAllPaths() {
        operator.clearPaths();
    }

    /** Returns the total number of queued paths. */
    public int getPathCount() {
        return operator.getTotalPaths();
    }

    /** Returns the index of the currently executing path. */
    public int getCurrentIndex() {
        return operator.getCurrentIndex();
    }

    /** Samples curvature values along the current trajectory. */
    public List<Double> getCurvatureSamples(int count) {
        return operator.getCurvatureSamples(count);
    }

    /** Samples heading angles along the current trajectory. */
    public List<Double> getHeadingSamples(int count) {
        return operator.getHeadingSamples(count);
    }

    /** Samples velocity vectors (forward, strafe, rotation) along the trajectory. */
    public List<org.firstinspires.ftc.teamcode.Pathing.PathingUtility.PathingVelocity> getVelocitySamples(int count) {
        return operator.getVelocitySamples(count);
    }

    /** Adds a prebuilt trajectory directly to the queue. */
    public void addTrajectory(Trajectory trajectory) {
        operator.addPath(trajectory);
    }

    public follower getFollower() {
        return operator.getFollower();
    }
}