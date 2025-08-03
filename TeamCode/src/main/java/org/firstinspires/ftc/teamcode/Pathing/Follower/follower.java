package org.firstinspires.ftc.teamcode.Pathing.Follower;

import org.firstinspires.ftc.teamcode.Pathing.Config.DriveConstants;
import org.firstinspires.ftc.teamcode.Pathing.Config.FieldConstants;
import org.firstinspires.ftc.teamcode.Pathing.Config.PIDConstants;
import org.firstinspires.ftc.teamcode.Pathing.Config.PathingConstants;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.Trajectory;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.PIDController;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.PathingPower;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.PathingVelocity;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

public class follower {
    private final Trajectory trajectory;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController headingController;
    private double startTime;

    // For acceleration clamping
    private double previousX = 0.0;
    private double previousY = 0.0;

    public follower(Trajectory trajectory) {
        this.trajectory = trajectory;
        this.xController = new PIDController(PIDConstants.X_KP, PIDConstants.X_KI, PIDConstants.X_KD);
        this.yController = new PIDController(PIDConstants.Y_KP, PIDConstants.Y_KI, PIDConstants.Y_KD);
        this.headingController = new PIDController(PIDConstants.HEADING_KP, PIDConstants.HEADING_KI, PIDConstants.HEADING_KD);
    }

    public void start(double currentTime) {
        this.startTime = currentTime;
    }

    public RobotPower update(Pose2d currentPose, double currentTime) {
        double t = currentTime - startTime;
        t = Math.min(t, trajectory.getTotalTime());

        Pose2d targetPose = trajectory.getPoseAt(t);
        PathingVelocity targetVelocity = trajectory.getVelocity(t);

        // Field bounds check
        if (targetPose.getX() < 0 || targetPose.getX() > FieldConstants.FIELD_WIDTH ||
                targetPose.getY() < 0 || targetPose.getY() > FieldConstants.FIELD_HEIGHT) {
            System.out.println("Warning: Target pose out of bounds");
        }

        double xError = targetPose.getX() - currentPose.getX();
        double yError = targetPose.getY() - currentPose.getY();
        double headingError = normalizeAngle(targetPose.getHeading() - currentPose.getHeading());

        double xCorrection = xController.calculate(xError);
        double yCorrection = yController.calculate(yError);
        double headingCorrection = headingController.calculate(headingError);

        double correctedX = targetVelocity.xVelocity + xCorrection;
        double correctedY = targetVelocity.yVelocity + yCorrection;
        double correctedRot = targetVelocity.rotationVelocity + headingCorrection;

        // Clamp to max velocity
        correctedX = clamp(correctedX, -DriveConstants.MAX_VELOCITY, DriveConstants.MAX_VELOCITY);
        correctedY = clamp(correctedY, -DriveConstants.MAX_VELOCITY, DriveConstants.MAX_VELOCITY);
        correctedRot = clamp(correctedRot, -PathingConstants.MAX_ROTATION_VELOCITY, PathingConstants.MAX_ROTATION_VELOCITY);

        // Clamp acceleration
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

        // Optional: Estimate angular velocity from linear motion
        double estimatedAngularVelocity = (correctedY - correctedX) / DriveConstants.TRACK_WIDTH;

        // Optional: Convert to motor ticks/sec (for logging or motor control)
        double ticksPerSecondX = inchesToTicks(correctedX);
        double ticksPerSecondY = inchesToTicks(correctedY);

        // Debug logging (can be toggled or routed to telemetry)
        System.out.printf("t=%.2f | X=%.2f Y=%.2f Rot=%.2f | TicksX=%.1f TicksY=%.1f | ω=%.2f\n",
                t, correctedX, correctedY, correctedRot, ticksPerSecondX, ticksPerSecondY, estimatedAngularVelocity);

        PathingPower controlOutput = new PathingPower(correctedY, correctedX, correctedRot);
        return RobotPower.fromPathingPower(controlOutput);
    }

    public boolean isFinished(double currentTime) {
        return currentTime - startTime >= trajectory.getTotalTime();
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double inchesToTicks(double inchesPerSecond) {
        return inchesPerSecond / (2 * Math.PI * DriveConstants.WHEEL_RADIUS)
                * DriveConstants.MOTOR_TICKS_PER_REV
                / DriveConstants.GEAR_RATIO;
    }
}