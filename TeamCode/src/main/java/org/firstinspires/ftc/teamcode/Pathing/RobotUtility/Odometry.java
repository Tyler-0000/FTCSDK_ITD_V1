package org.firstinspires.ftc.teamcode.Pathing.RobotUtility;

import org.firstinspires.ftc.teamcode.Pathing.Config.OdometryConstants;
import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;

public class Odometry {
    private final OdometryHardware hardware;
    private Pose2d pose = new Pose2d(0, 0, 0);
    private double lastLeftTicks = 0, lastRightTicks = 0, lastStrafeTicks = 0;
    private double leftTicks = 0, rightTicks = 0, strafeTicks = 0;

    public Odometry(OdometryHardware hardware) {
        this.hardware = hardware;
    }

    /**
     * Updates the robot's pose using encoder deltas.
     * Heading is inferred from left/right wheel deltas.
     * STRAFE_OFFSET corrects lateral drift caused by rotation.
     */
    public void update() {

        lastLeftTicks = leftTicks;
        lastRightTicks = rightTicks;
        lastStrafeTicks = strafeTicks;

        leftTicks = hardware.getLeftTicks();
        rightTicks = hardware.getRightTicks();
        strafeTicks = hardware.getStrafeTicks();

        double dLeft = ticksToInches(leftTicks - lastLeftTicks);
        double dRight = ticksToInches(rightTicks - lastRightTicks);
        double dStrafeRaw = ticksToInches(strafeTicks - lastStrafeTicks);

        double dForward = (dLeft + dRight) / 2.0;
        double deltaHeading = (dRight - dLeft) / OdometryConstants.TRACK_WIDTH;
        double newHeading = normalizeAngle(pose.getHeading() + deltaHeading);

        // Correct strafe for rotational contribution
        double dStrafe = dStrafeRaw - deltaHeading * OdometryConstants.STRAFE_OFFSET;

        // Transform local deltas into global field coordinates
        double dx = dForward * Math.cos(newHeading) - dStrafe * Math.sin(newHeading);
        double dy = dForward * Math.sin(newHeading) + dStrafe * Math.cos(newHeading);

        pose = new Pose2d(pose.getX() + dx, pose.getY() + dy, newHeading);
    }

    /**
     * Returns the current estimated pose.
     */
    public Pose2d getPose() {
        return pose;
    }

    /**
     * Resets the pose and synchronizes encoder baselines.
     */
    public void reset(Pose2d newPose) {
        pose = newPose;
        lastLeftTicks = hardware.getLeftTicks();
        lastRightTicks = hardware.getRightTicks();
        lastStrafeTicks = hardware.getStrafeTicks();
    }

    /**
     * Converts encoder ticks to inches.
     */
    private double ticksToInches(double ticks) {
        return ticks / OdometryConstants.TICKS_PER_REV
                * OdometryConstants.WHEEL_CIRCUMFERENCE
                * OdometryConstants.GEAR_RATIO;
    }

    /**
     * Normalizes angle to [-π, π] range.
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}