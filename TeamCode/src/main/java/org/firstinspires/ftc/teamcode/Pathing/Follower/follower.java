package org.firstinspires.ftc.teamcode.Pathing.Follower;

import org.firstinspires.ftc.teamcode.Pathing.Config.DriveConstants;
import org.firstinspires.ftc.teamcode.Pathing.Config.PIDConstants;
import org.firstinspires.ftc.teamcode.Pathing.Config.PathingConstants;
import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.Trajectory;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.*;
import org.firstinspires.ftc.teamcode.Pathing.Optimization.Efficiency.EfficiencyModel;
import org.firstinspires.ftc.teamcode.Pathing.Optimization.Speed.SpeedOptimizer;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.*;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Odometry;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

/**
 * Follows a trajectory using PID correction and velocity blending.
 * Assumes:
 *   - x = forward
 *   - y = strafe
 */
public class follower {

    private final Trajectory trajectory;
    private final Odometry odometry;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController headingController;
    private final PIDController finalXController;
    private final PIDController finalYController;

    private double previousX = 0.0;
    private double previousY = 0.0;

    private boolean holdPosition = false;
    private boolean useRobotCentricCorrection = false;
    private boolean useHeadingLookahead = false;
    private boolean useVelocityGatedIntegral = true;

    private double xIntegral = 0.0;
    private double yIntegral = 0.0;

    public follower(Trajectory trajectory, Odometry odometry) {
        this.trajectory = trajectory;
        this.odometry = odometry;

        this.xController = new PIDController(PIDConstants.X_KP, PIDConstants.X_KI, PIDConstants.X_KD);
        this.yController = new PIDController(PIDConstants.Y_KP, PIDConstants.Y_KI, PIDConstants.Y_KD);
        this.headingController = new PIDController(PIDConstants.HEADING_KP, PIDConstants.HEADING_KI, PIDConstants.HEADING_KD);

        this.finalXController = new PIDController(PIDConstants.X_KP_END, 0, PIDConstants.X_KD_END);
        this.finalYController = new PIDController(PIDConstants.Y_KP_END, 0, PIDConstants.Y_KD_END);
    }

    public RobotPower update() {
        odometry.update();
        Pose2d currentPose = odometry.getPose();
        Pose2d targetPose = trajectory.getClosestPose(currentPose);
        PathingVelocity targetVelocity = trajectory.getVelocityAtPose(targetPose);

        Pose2d lookaheadPose = trajectory.getLookaheadPose(currentPose, 0.3);
        double headingTarget = useHeadingLookahead ? lookaheadPose.heading : targetPose.heading;

        // Error in robot-centric coordinates
        double forwardError = targetPose.x - currentPose.x; // x = forward
        double strafeError = targetPose.y - currentPose.y;  // y = strafe
        double headingError = normalizeAngle(headingTarget - currentPose.heading);

        boolean nearEnd = trajectory.isNearEnd(currentPose);
        boolean lowVelocity = Math.abs(targetVelocity.xVelocity) < 3 && Math.abs(targetVelocity.yVelocity) < 3;

        // Velocity-gated integral accumulation
        if (useVelocityGatedIntegral && lowVelocity) {
            xIntegral = Math.abs(forwardError) > 1 ? xIntegral + 0.002 : 0;
            yIntegral = Math.abs(strafeError) > 1 ? yIntegral + 0.002 : 0;
        } else {
            xIntegral = 0;
            yIntegral = 0;
        }

        finalXController.setI(xIntegral);
        finalYController.setI(yIntegral);

        double correctedX, correctedY, correctedRot;

        if (trajectory.isFinished(currentPose) && holdPosition) {
            correctedX = finalXController.calculate(forwardError);
            correctedY = finalYController.calculate(strafeError);
            correctedRot = headingController.calculate(headingError);
        } else {
            double xCorrection = nearEnd ? finalXController.calculate(forwardError) : xController.calculate(forwardError);
            double yCorrection = nearEnd ? finalYController.calculate(strafeError) : yController.calculate(strafeError);
            double headingCorrection = headingController.calculate(headingError);

            correctedX = targetVelocity.xVelocity + xCorrection; // forward
            correctedY = targetVelocity.yVelocity + yCorrection; // strafe
            correctedRot = targetVelocity.rotationVelocity + headingCorrection;
        }

        // Optional robot-centric correction
        if (useRobotCentricCorrection) {
            double sin = Math.sin(currentPose.heading);
            double cos = Math.cos(currentPose.heading);
            double tempX = correctedX * cos - correctedY * sin;
            double tempY = correctedX * sin + correctedY * cos;
            correctedX = tempX;
            correctedY = tempY;
        }

        // Clamp to velocity limits
        correctedX = clamp(correctedX, -DriveConstants.MAX_VELOCITY, DriveConstants.MAX_VELOCITY);
        correctedY = clamp(correctedY, -DriveConstants.MAX_VELOCITY, DriveConstants.MAX_VELOCITY);
        correctedRot = clamp(correctedRot, -PathingConstants.MAX_ROTATION_VELOCITY, PathingConstants.MAX_ROTATION_VELOCITY);

        // Acceleration limiting
        double dt = PathingConstants.SAMPLE_RESOLUTION;
        double accelX = (correctedX - previousX) / dt;
        double accelY = (correctedY - previousY) / dt;

        if (Math.abs(accelX) > DriveConstants.MAX_ACCELERATION) {
            correctedX = previousX + Math.signum(accelX) * DriveConstants.MAX_ACCELERATION * dt;
        }
        if (Math.abs(accelY) > DriveConstants.MAX_ACCELERATION) {
            correctedY = previousY + Math.signum(accelY) * DriveConstants.MAX_ACCELERATION * dt;
        }

        previousX = correctedX;
        previousY = correctedY;

        // Output format: (strafe, forward, rotation)
        PathingPower controlOutput = new PathingPower(correctedY, correctedX, correctedRot);
        return RobotPower.fromPathingPower(controlOutput);
    }

    public void applyToHardware(OdometryHardware hardware,
                                CorrectionStrategy strategy,
                                CorrectionScaler scaler,
                                CorrectionBlender blender,
                                DiagnosticLogger logger,
                                EfficiencyModel efficiencyModel,
                                SpeedOptimizer speedOptimizer,
                                double batteryVoltage) {
        Pose2d currentPose = odometry.getPose();
        Pose2d targetPose = trajectory.getClosestPose(currentPose);
        RobotPower basePower = update();

        hardware.applyPower(
                basePower,
                targetPose,
                currentPose,
                PathingConstants.SAMPLE_RESOLUTION,
                strategy,
                scaler,
                blender,
                logger,
                efficiencyModel,
                speedOptimizer,
                batteryVoltage
        );
    }

    public void setHoldPosition(boolean hold) {
        this.holdPosition = hold;
    }

    public void setUseRobotCentricCorrection(boolean useRobotCentric) {
        this.useRobotCentricCorrection = useRobotCentric;
    }

    public void setUseHeadingLookahead(boolean useLookahead) {
        this.useHeadingLookahead = useLookahead;
    }

    public void setUseVelocityGatedIntegral(boolean useGating) {
        this.useVelocityGatedIntegral = useGating;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public Pose2d getTargetPose() {
        return trajectory.getClosestPose(odometry.getPose());
    }

    public Pose2d getLookaheadPose() {
        return trajectory.getLookaheadPose(odometry.getPose(), 0.3);
    }

    public Pose2d getPoseError() {
        Pose2d currentPose = odometry.getPose();
        Pose2d targetPose = trajectory.getClosestPose(currentPose);
        return new Pose2d(
                targetPose.x - currentPose.x,     // forward error
                targetPose.y - currentPose.y,     // strafe error
                normalizeAngle(targetPose.heading - currentPose.heading)
        );
    }

    public RobotPower getCurrentPower() {
        return update();
    }

    public boolean isFinished() {
        return trajectory.isFinished(odometry.getPose());
    }

    public boolean isNearEnd() {
        return trajectory.isNearEnd(odometry.getPose());
    }

    public double getProgress() {
        double distance = trajectory.getDistanceAtPose(odometry.getPose());
        return distance / trajectory.getTotalLength();
    }

    public double getCurvature() {
        double distance = trajectory.getDistanceAtPose(odometry.getPose());
        return trajectory.getCurvatureAtDistance(distance);
    }

    public PathingVelocity getCurrentVelocity() {
        return trajectory.getVelocityAtPose(odometry.getPose());
    }

    public PathingVelocity getTargetVelocity() {
        Pose2d targetPose = trajectory.getClosestPose(odometry.getPose());
        return trajectory.getVelocityAtPose(targetPose);
    }
}