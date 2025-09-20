package org.firstinspires.ftc.teamcode.Robot.Test.PathingTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.CorrectionBlender;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.CorrectionScaler;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.CorrectionStrategy;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.DiagnosticLogger;
import org.firstinspires.ftc.teamcode.Pathing.Optimization.Efficiency.EfficiencyModel;
import org.firstinspires.ftc.teamcode.Pathing.Optimization.Speed.SpeedOptimizer;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation.PIDCorrectionStrategy;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation.SimpleScalar;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation.WeightedBlender;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Odometry;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

@Autonomous(name = "TestKeepPositionAuto")
public class TestKeepPositionAuto extends LinearOpMode {

    private static final double TARGET_DISTANCE = 0;
    private static final double POSITION_THRESHOLD = 1.0;
    private static final double FORWARD_SPEED = 1.0;

    private static final double STALL_TICKS_THRESHOLD = 5.0;
    private static final double STALL_POWER_THRESHOLD = 0.2;

    private Pose2d targetPose;
    private boolean reachedTarget = false;

    private double lastTime = 0.0;

    @Override
    public void runOpMode() {
        OdometryHardware hardware = new OdometryHardware(hardwareMap);
        Odometry odometry = new Odometry(hardware);

        Pose2d startPose = new Pose2d(0, 0, 0);
        odometry.reset(startPose);
        targetPose = new Pose2d(TARGET_DISTANCE, 0, 0);

        telemetry.addLine("Ready to test keep-position behavior");
        telemetry.update();
        waitForStart();

        lastTime = getRuntime();

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            odometry.update();
            Pose2d currentPose = odometry.getPose();

            double distance = currentPose.distanceTo(targetPose);
            reachedTarget = distance < POSITION_THRESHOLD;

            RobotPower basePower = reachedTarget
                    ? RobotPower.zero()
                    : new RobotPower(FORWARD_SPEED, 0.0, 0.0);

            hardware.applyPower(
                    RobotPower.zero(), // basePower is zero; let correction drive everything
                    targetPose,
                    currentPose,
                    dt,
                    new PIDCorrectionStrategy(),
                    new SimpleScalar(0.8, 0.8, 1.0),
                    new WeightedBlender(0.0, 1.0),
                    (c, t, r, s, f) -> {}, // DiagnosticLogger
                    (p, pose, v) -> p,     // EfficiencyModel
                    (p, pose, v) -> p,     // SpeedOptimizer
                    12.0
            );

            if (hardware.isStalled(STALL_TICKS_THRESHOLD, STALL_POWER_THRESHOLD)) {
                telemetry.addLine("Stall detected — attempting recovery");
                hardware.attemptStallRecovery();
            }

            telemetry.addData("Pose", currentPose);
            telemetry.addData("Target", targetPose);
            telemetry.addData("Distance to Target", distance);
            telemetry.addData("Reached Target", reachedTarget);
            telemetry.addData("Power", basePower);
            telemetry.update();
        }

        hardware.stop();
        telemetry.addLine("Keep-position test complete");
        telemetry.update();
    }
}