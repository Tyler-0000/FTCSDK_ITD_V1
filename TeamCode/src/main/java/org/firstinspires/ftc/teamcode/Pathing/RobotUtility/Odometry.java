package org.firstinspires.ftc.teamcode.Pathing.RobotUtility;

import org.firstinspires.ftc.teamcode.Pathing.Config.OdometryConstants;
import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;

public class Odometry {
    private final OdometryHardware hardware;
    public Pose2d pose = new Pose2d(0, 0, 0); // mutable pose

    private int currentLeftTicks = 0;
    private int currentRightTicks = 0;
    private int currentStrafeTicks = 0;

    private int previousLeftTicks = 0;
    private int previousRightTicks = 0;
    private int previousStrafeTicks = 0;

    public Odometry(OdometryHardware hardware) {
        this.hardware = hardware;
    }

    /**
     * Updates the robot's pose using encoder deltas.
     * Heading is inferred from left/right wheel deltas.
     * Strafe offset corrects lateral drift caused by rotation.
     */
    public void update() {
        // Save previous encoder values
        previousLeftTicks = currentLeftTicks;
        previousRightTicks = currentRightTicks;
        previousStrafeTicks = currentStrafeTicks;

        // Read current encoder values
        currentLeftTicks = -hardware.getLeftTicks();
        currentRightTicks = hardware.getRightTicks();
        currentStrafeTicks = hardware.getStrafeTicks();

        // Calculate deltas
        int dn1 = currentLeftTicks - previousLeftTicks;
        int dn2 = currentRightTicks - previousRightTicks;
        int dn3 = currentStrafeTicks - previousStrafeTicks;

        // Convert ticks to inches
        double dtheta = ticksToInches(dn2 - dn1) / OdometryConstants.TRACK_WIDTH;  //dstance / diamer => radius
        double dx = ticksToInches((dn1 + dn2) / 2.0);
        double dy = ticksToInches(dn3 + ((dn2 - dn1) / 2.0));

        // Midpoint heading update
        pose.heading -= dtheta / 2.0;               //

        // Field-centric transformation (swapped x and y)
        pose.x += dx * Math.cos(pose.heading) - dy * Math.sin(pose.heading);
        pose.y -= dx * Math.sin(pose.heading) + dy * Math.cos(pose.heading);

        //- instead of +
        pose.heading -= dtheta / 2.0;

        normalizeHeading();
    }

    /**
     * Returns the current estimated pose with optional scaling.
     */
    public Pose2d getPose() {
        return new Pose2d(
                pose.x * OdometryConstants.TEST_X_OFFSET,
                pose.y * OdometryConstants.TEST_Y_OFFSET,
                pose.heading * OdometryConstants.TEST_HEADING_OFFSET
        );
    }

    /**
     * Resets the pose and synchronizes encoder baselines.
     */
    public void reset(Pose2d newPose) {
        pose.x = newPose.x;
        pose.y = newPose.y;
        pose.heading = newPose.heading;
        normalizeHeading();

        currentLeftTicks = -hardware.getLeftTicks();
        currentRightTicks = hardware.getRightTicks();
        currentStrafeTicks = hardware.getStrafeTicks();

        previousLeftTicks = currentLeftTicks;
        previousRightTicks = currentRightTicks;
        previousStrafeTicks = currentStrafeTicks;
    }

    /**
     * Converts encoder ticks to inches.
     */
    public double ticksToInches(double ticks) {
        return ticks / OdometryConstants.TICKS_PER_REV
                * OdometryConstants.WHEEL_CIRCUMFERENCE
                * OdometryConstants.GEAR_RATIO;
    }

    /**
     * Normalizes heading to [-π, π].
     */
    private void normalizeHeading() {
        while (pose.heading > Math.PI) pose.heading -= 2 * Math.PI;
        while (pose.heading < -Math.PI) pose.heading += 2 * Math.PI;
    }
}