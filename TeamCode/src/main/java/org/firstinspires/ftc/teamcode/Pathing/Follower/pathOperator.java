package org.firstinspires.ftc.teamcode.Pathing.Follower;

import org.firstinspires.ftc.teamcode.Pathing.Config.FieldConstants;
import org.firstinspires.ftc.teamcode.Pathing.Config.PathingConstants;
import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.Trajectory;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.TrajectoryDebugger;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.*;
import org.firstinspires.ftc.teamcode.Pathing.Optimization.Efficiency.EfficiencyModel;
import org.firstinspires.ftc.teamcode.Pathing.Optimization.Speed.SpeedOptimizer;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.PathingVelocity;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Odometry;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Manages path execution and diagnostics for a queue of trajectories.
 * Assumes:
 *   - x = forward
 *   - y = strafe
 */
public class pathOperator {

    private final List<Trajectory> pathQueue = new ArrayList<>();
    private follower currentFollower = null;
    private int currentIndex = -1;

    private final Odometry odometry;
    private final OdometryHardware hardware;
    private final Telemetry telemetry;
    private final TrajectoryDebugger debugger;

    private final CorrectionStrategy strategy;
    private final CorrectionScaler scaler;
    private final CorrectionBlender blender;
    private final DiagnosticLogger logger;
    private final EfficiencyModel efficiencyModel;
    private final SpeedOptimizer speedOptimizer;

    private double batteryVoltage = 12.0;

    public pathOperator(Odometry odometry,
                        OdometryHardware hardware,
                        Telemetry telemetry,
                        CorrectionStrategy strategy,
                        CorrectionScaler scaler,
                        CorrectionBlender blender,
                        DiagnosticLogger logger,
                        EfficiencyModel efficiencyModel,
                        SpeedOptimizer speedOptimizer) {
        this.odometry = odometry;
        this.hardware = hardware;
        this.telemetry = telemetry;
        this.debugger = new TrajectoryDebugger();

        this.strategy = strategy;
        this.scaler = scaler;
        this.blender = blender;
        this.logger = logger;
        this.efficiencyModel = efficiencyModel;
        this.speedOptimizer = speedOptimizer;
    }

    public void setBatteryVoltage(double voltage) {
        this.batteryVoltage = voltage;
    }

    /** Adds a trajectory to the queue if it's valid and in bounds. */
    public void addPath(Trajectory trajectory) {
        if (trajectory.getTotalLength() < PathingConstants.MIN_PATH_LENGTH) {
            telemetry.addLine("Skipping short path");
            telemetry.update();
            return;
        }

        Pose2d endPose = trajectory.getPoseAtDistance(trajectory.getTotalLength());
        if (!isPoseInBounds(endPose)) {
            telemetry.addLine("Rejected path: end pose out of bounds");
            telemetry.update();
            return;
        }

        pathQueue.add(trajectory);
    }

    /** Starts execution of the trajectory at the given index. */
    public void startPath(int index) {
        if (index >= 0 && index < pathQueue.size()) {
            currentIndex = index;
            Trajectory traj = pathQueue.get(index);
            currentFollower = new follower(traj, odometry);
            debugger.sampleExpected(traj, 100);

            telemetry.addLine("Starting path " + index);
            telemetry.update();

            currentFollower.applyToHardware(
                    hardware,
                    strategy,
                    scaler,
                    blender,
                    logger,
                    efficiencyModel,
                    speedOptimizer,
                    batteryVoltage
            );
        }
    }

    /** Updates the follower and logs the current pose. */
    public void updateAndDrive() {
        if (currentFollower == null) return;

        debugger.logActual(odometry.getPose());

        currentFollower.applyToHardware(
                hardware,
                strategy,
                scaler,
                blender,
                logger,
                efficiencyModel,
                speedOptimizer,
                batteryVoltage
        );
    }

    /** Returns true if the current path is finished. */
    public boolean isCurrentPathFinished() {
        boolean finished = currentFollower != null && currentFollower.isFinished();
        if (finished) {
            currentFollower.setHoldPosition(true);
            debugger.comparePaths(telemetry);
        }
        return finished;
    }

    public int getCurrentIndex() {
        return currentIndex;
    }

    public int getTotalPaths() {
        return pathQueue.size();
    }

    public void clearPaths() {
        pathQueue.clear();
        currentFollower = null;
        currentIndex = -1;
    }

    public Pose2d getClosestPose(Pose2d currentPose) {
        if (!isValidIndex()) return new Pose2d(0, 0, 0);
        return pathQueue.get(currentIndex).getClosestPose(currentPose);
    }

    public double getPathLength() {
        return isValidIndex() ? pathQueue.get(currentIndex).getTotalLength() : 0;
    }

    public double getProgress(Pose2d currentPose) {
        if (!isValidIndex()) return 0;
        Trajectory traj = pathQueue.get(currentIndex);
        double distance = traj.getDistanceAtPose(currentPose);
        return distance / traj.getTotalLength();
    }

    public List<Double> getCurvatureSamples(int count) {
        return isValidIndex() ? pathQueue.get(currentIndex).getCurvatureList(count) : new ArrayList<>();
    }

    public List<Double> getHeadingSamples(int count) {
        return isValidIndex() ? pathQueue.get(currentIndex).getHeadingList(count) : new ArrayList<>();
    }

    public List<PathingVelocity> getVelocitySamples(int count) {
        return isValidIndex() ? pathQueue.get(currentIndex).getVelocityList(count) : new ArrayList<>();
    }

    public Pose2d getPoseError() {
        return currentFollower != null ? currentFollower.getPoseError() : new Pose2d(0, 0, 0);
    }

    public boolean isDeviating() {
        Pose2d error = getPoseError();
        double dist = Math.hypot(error.x, error.y); // forward/strafe deviation
        return dist > PathingConstants.DEVIATION_THRESHOLD;
    }

    public RobotPower getCurrentPower() {
        return currentFollower != null ? currentFollower.getCurrentPower() : RobotPower.zero();
    }

    public Pose2d getCurrentPose() {
        return odometry.getPose();
    }

    private boolean isPoseInBounds(Pose2d pose) {
        return pose.x >= 0 && pose.x <= FieldConstants.FIELD_WIDTH &&
                pose.y >= 0 && pose.y <= FieldConstants.FIELD_HEIGHT;
    }

    private boolean isValidIndex() {
        return currentIndex >= 0 && currentIndex < pathQueue.size();
    }

    public follower getFollower() {
        return currentFollower;
    }
}