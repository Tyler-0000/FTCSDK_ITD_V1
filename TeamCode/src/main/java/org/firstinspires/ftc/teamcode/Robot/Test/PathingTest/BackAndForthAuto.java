package org.firstinspires.ftc.teamcode.Robot.Test.PathingTest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.CorrectionBlender;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.CorrectionScaler;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.CorrectionStrategy;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation.PIDCorrectionStrategy;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation.SimpleScalar;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation.WeightedBlender;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Odometry;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

@Autonomous(name = "BackAndForthModeled", group = "Pathing Tests")
public class BackAndForthAuto extends OpMode {
    private MultipleTelemetry telemetryA;

    private static final double DISTANCE = 24.0;
    private static final double POSITION_THRESHOLD = 1.0;
    private static final int STABLE_CYCLES = 3;

    private boolean goingForward = true;
    private int stableCount = 0;

    private OdometryHardware hardware;
    private Odometry odometry;

    private Pose2d origin = new Pose2d(0, 0, 0);
    private Pose2d forwardPose = new Pose2d(DISTANCE, 0, 0);
    private Pose2d targetPose;

    private CorrectionStrategy strategy;
    private CorrectionScaler scaler;
    private CorrectionBlender blender;

    private double lastTime;

    @Override
    public void init() {
        hardware = new OdometryHardware(hardwareMap);
        odometry = new Odometry(hardware);
        odometry.reset(origin);

        strategy = new PIDCorrectionStrategy();
        scaler = new SimpleScalar(0.8, 0.8, 1.0);
        blender = new WeightedBlender(0.7, 0.5);

        targetPose = forwardPose;

        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("Back-and-forth initialized. Robot will move " + DISTANCE + " inches forward and back.");
        telemetryA.update();
    }

    @Override
    public void start() {
        lastTime = getRuntime();
    }

    @Override
    public void loop() {
        double currentTime = getRuntime();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        odometry.update();
        Pose2d currentPose = odometry.getPose();

        double distance = currentPose.distanceTo(targetPose);
        boolean reachedTarget = distance < POSITION_THRESHOLD;

        if (reachedTarget) {
            stableCount++;
            if (stableCount >= STABLE_CYCLES) {
                goingForward = !goingForward;
                targetPose = goingForward ? forwardPose : origin;
                stableCount = 0;
            }
        } else {
            stableCount = 0;
        }

        RobotPower basePower = reachedTarget ? RobotPower.zero()
                : computeBasePower(currentPose, targetPose);

        hardware.applyPower(
                basePower,
                targetPose,
                currentPose,
                dt,
                strategy,
                scaler,
                blender,
                (c, t, r, s, f) -> {}, // DiagnosticLogger
                (p, pose, v) -> p,     // EfficiencyModel
                (p, pose, v) -> p,     // SpeedOptimizer
                12.0
        );

        telemetryA.addData("Pose", currentPose);
        telemetryA.addData("Target", targetPose);
        telemetryA.addData("Distance to Target", distance);
        telemetryA.addData("Going Forward", goingForward);
        telemetryA.addData("Stable Count", stableCount);
        telemetryA.update();
    }

    private RobotPower computeBasePower(Pose2d current, Pose2d target) {
        double direction = Math.signum(target.x - current.x);
        return new RobotPower(direction, 0.0, 0.0);
    }
}