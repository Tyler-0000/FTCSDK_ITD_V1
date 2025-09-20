package org.firstinspires.ftc.teamcode.Pathing.Follower.Abstracted;

import org.firstinspires.ftc.teamcode.Pathing.Follower.follower;
import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.Trajectory;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.*;
import org.firstinspires.ftc.teamcode.Pathing.Optimization.Efficiency.EfficiencyModel;
import org.firstinspires.ftc.teamcode.Pathing.Optimization.Speed.SpeedOptimizer;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Odometry;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

/**
 * High-level controller for executing a single trajectory.
 * Wraps a follower instance and exposes control hooks.
 * Assumes:
 *   - x = forward
 *   - y = strafe
 */
public class FollowerController {

    private final follower internalFollower;

    public FollowerController(Trajectory trajectory,
                              OdometryHardware hardware,
                              CorrectionStrategy strategy,
                              CorrectionScaler scaler,
                              CorrectionBlender blender,
                              DiagnosticLogger logger,
                              EfficiencyModel efficiencyModel,
                              SpeedOptimizer speedOptimizer,
                              double batteryVoltage) {

        Odometry odometry = new Odometry(hardware); // Constructs odometry from hardware
        this.internalFollower = new follower(trajectory, odometry);

        apply(hardware, strategy, scaler, blender, logger, efficiencyModel, speedOptimizer, batteryVoltage);
    }

    /** Updates internal follower logic (pose tracking, PID correction). */
    public void update() {
        internalFollower.update();
    }

    /** Applies control output to hardware using configured correction stack. */
    public void apply(OdometryHardware hardware,
                      CorrectionStrategy strategy,
                      CorrectionScaler scaler,
                      CorrectionBlender blender,
                      DiagnosticLogger logger,
                      EfficiencyModel efficiencyModel,
                      SpeedOptimizer speedOptimizer,
                      double batteryVoltage) {
        internalFollower.applyToHardware(
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

    /** Enables or disables final position holding after trajectory completion. */
    public void setHoldPosition(boolean hold) {
        internalFollower.setHoldPosition(hold);
    }

    /** Enables robot-centric correction (rotates error vector into robot frame). */
    public void setUseRobotCentricCorrection(boolean useRobotCentric) {
        internalFollower.setUseRobotCentricCorrection(useRobotCentric);
    }

    /** Enables heading lookahead (uses future heading instead of current target). */
    public void setUseHeadingLookahead(boolean useLookahead) {
        internalFollower.setUseHeadingLookahead(useLookahead);
    }

    /** Enables velocity-gated integral accumulation for low-speed correction. */
    public void setUseVelocityGatedIntegral(boolean useGating) {
        internalFollower.setUseVelocityGatedIntegral(useGating);
    }

    /** Returns the current target pose (closest point on trajectory). */
    public Pose2d getTargetPose() {
        return internalFollower.getTargetPose(); // x = forward, y = strafe
    }

    /** Returns the lookahead pose used for heading prediction. */
    public Pose2d getLookaheadPose() {
        return internalFollower.getLookaheadPose(); // x = forward, y = strafe
    }

    /** Returns the pose error in (forward, strafe, heading). */
    public Pose2d getPoseError() {
        return internalFollower.getPoseError(); // x = forward error, y = strafe error
    }

    /** Returns the current power output in robot-centric coordinates. */
    public RobotPower getCurrentPower() {
        return internalFollower.getCurrentPower(); // forward, strafe, rotation
    }

    /** Returns true if the trajectory has been completed. */
    public boolean isFinished() {
        return internalFollower.isFinished();
    }

    /** Returns true if the robot is near the end of the trajectory. */
    public boolean isNearEnd() {
        return internalFollower.isNearEnd();
    }

    /** Returns progress along the trajectory (0.0 to 1.0). */
    public double getProgress() {
        return internalFollower.getProgress();
    }

    /** Returns curvature at the robot's current position on the trajectory. */
    public double getCurvature() {
        return internalFollower.getCurvature();
    }
}