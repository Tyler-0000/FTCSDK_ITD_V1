package org.firstinspires.ftc.teamcode.Robot.Test.PathingTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pathing.Follower.Abstracted.PathManager;
import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathBuilder;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.CorrectionStrategy;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation.PIDCorrectionStrategy;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation.SimpleScalar;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation.WeightedBlender;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Odometry;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

import java.util.List;

@Autonomous(name = "KeepPositionTest")
public class KeepPositionTest extends LinearOpMode {
    private static final double STALL_TICKS_THRESHOLD = 5.0;
    private static final double STALL_POWER_THRESHOLD = 0.2;

    @Override
    public void runOpMode() {
        OdometryHardware hardware = new OdometryHardware(hardwareMap);
        Odometry odometry = new Odometry(hardware);

        PathManager pathManager = new PathManager(
                hardware,
                telemetry,
                new PIDCorrectionStrategy(),                     // Full PID correction
                new SimpleScalar(0.8, 0.8, 1.0),                 // Scales forward, strafe, turn corrections
                new WeightedBlender(0.0, 1.0),                   // Uses only correction (no base power)
                (current, target, raw, scaled, finalPower) -> {},// No-op DiagnosticLogger
                (power, pose, voltage) -> power,                 // EfficiencyModel passthrough
                (power, pose, voltage) -> power                  // SpeedOptimizer passthrough
        );
        // Define a single pose to hold
        Pose2d holdPose = new Pose2d(1, 0, 0); // x = forward, y = strafe, heading = 0

        // Build a linear path with just one pose
        pathManager.addLinearPath(List.of(holdPose), HybridPathBuilder.SegmentType.BEZIER, 10.0, false);

        telemetry.addLine("Ready to hold position");
        telemetry.update();
        waitForStart();

        pathManager.startPath(0);

        double startTime = getRuntime();
        double lastTime = startTime;

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            odometry.update();
            Pose2d currentPose = odometry.getPose();

            pathManager.update();

            if (hardware.isStalled(STALL_TICKS_THRESHOLD, STALL_POWER_THRESHOLD)) {
                telemetry.addLine("Stall detected — attempting recovery");
                hardware.attemptStallRecovery();
            }

            telemetry.addData("Pose Error", pathManager.getPoseError());
            telemetry.addData("Current Pose", currentPose);
            Pose2d error = pathManager.getPoseError();
            telemetry.addData("Forward Error (x)", error.x);
            telemetry.addData("Strafe Error (y)", error.y);
            telemetry.addData("Heading Error (rad)", error.heading);
            telemetry.update();
        }

        hardware.stop();
        telemetry.addLine("Hold complete");
        telemetry.update();
    }
}
