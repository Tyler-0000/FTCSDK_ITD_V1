package org.firstinspires.ftc.teamcode.Robot.Test.PathingTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.*;
import org.firstinspires.ftc.teamcode.Pathing.Optimization.Efficiency.EfficiencyModel;
import org.firstinspires.ftc.teamcode.Pathing.Optimization.Speed.SpeedOptimizer;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Odometry;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

@Autonomous(name = "SpinTestAuto")
public class SpinTestAuto extends LinearOpMode {

    private static final double TARGET_HEADING = 2 * Math.PI; // Full rotation
    private static final double ROTATION_SPEED = 1.0;          // rad/sec
    private static final double HEADING_THRESHOLD = 0.05;      // rad

    @Override
    public void runOpMode() {
        OdometryHardware hardware = new OdometryHardware(hardwareMap);
        Odometry odometry = new Odometry(hardware);

        Pose2d startPose = new Pose2d(0, 0, 0);
        odometry.reset(startPose);

        telemetry.addLine("Ready to run spin test");
        telemetry.update();
        waitForStart();

        double lastTime = getRuntime();

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            odometry.update();
            Pose2d currentPose = odometry.getPose();

            double headingError = normalizeAngle(TARGET_HEADING - currentPose.heading);
            boolean reachedTarget = Math.abs(headingError) < HEADING_THRESHOLD;

            RobotPower basePower = reachedTarget
                    ? RobotPower.zero()
                    : new RobotPower(0.0, 0.0, Math.signum(headingError) * ROTATION_SPEED);

            hardware.applyPower(
                    basePower,
                    new Pose2d(0, 0, TARGET_HEADING),
                    currentPose,
                    dt,
                    (t, c, d) -> RobotPower.zero(),
                    (r, p, d) -> r,
                    (b, c) -> b,
                    (c, t, r, s, f) -> {},
                    (p, pose, v) -> p,
                    (p, pose, v) -> p,
                    12.0
            );

            telemetry.addData("Current Heading", currentPose.heading);
            telemetry.addData("Target Heading", TARGET_HEADING);
            telemetry.addData("Heading Error", headingError);
            telemetry.addData("Power", basePower);
            telemetry.update();

            if (reachedTarget) break;
        }

        hardware.stop();
        telemetry.addLine("Spin test complete");
        telemetry.update();
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}