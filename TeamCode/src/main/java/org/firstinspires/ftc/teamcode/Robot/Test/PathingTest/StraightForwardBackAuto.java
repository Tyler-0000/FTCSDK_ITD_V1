package org.firstinspires.ftc.teamcode.Robot.Test.PathingTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pathing.Follower.Abstracted.PathManager;
import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathBuilder;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.*;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation.PIDCorrectionStrategy;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation.SimpleScalar;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation.WeightedBlender;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Odometry;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

import java.util.List;

/**
 * Autonomous test that drives forward and back along a linear path.
 * Assumes:
 *   - x = forward
 *   - y = strafe
 */
@Autonomous(name = "StraightForwardBackAuto")
public class StraightForwardBackAuto extends LinearOpMode {

    private static final double HOLD_DURATION = 2.0;     // seconds to hold final pose
    private static final double CORRECTION_GAIN = 0.1;   // proportional gain for simple correction

    @Override
    public void runOpMode() {
        OdometryHardware hardware = new OdometryHardware(hardwareMap);
        Odometry odometry = new Odometry(hardware);

        // Simple proportional correction strategy (x = forward, y = strafe)
        CorrectionStrategy simpleCorrection = (target, current, dt) -> {
            double forwardError = target.x - current.x;   // x is forward
            double strafeError = target.y - current.y;    // y is strafe
            double headingError = normalizeAngle(target.heading - current.heading);

            return new RobotPower(
                    forwardError * CORRECTION_GAIN,   // forward
                    strafeError * CORRECTION_GAIN,    // strafe
                    headingError * CORRECTION_GAIN
            );
        };

        PathManager pathManager = new PathManager(
                hardware,
                telemetry,
                new PIDCorrectionStrategy(),
                new SimpleScalar(0.8, 0.8, 1.0),
                new WeightedBlender(0.0, 1.0),
                (current, target, raw, scaled, finalPower) -> {
                    telemetry.addData("Raw Correction", raw);
                    telemetry.addData("Scaled Correction", scaled);
                    telemetry.addData("Final Power", finalPower);
                },
                (power, pose, voltage) -> power,
                (power, pose, voltage) -> power
        );

        // Define start and end poses (x = forward, y = strafe)
        Pose2d origin = new Pose2d(0, 0, 0); // heading along x-axis
        Pose2d top = new Pose2d(12, 0, 0);   // move forward along x

        // Add forward and reverse paths
        pathManager.addLinearPath(List.of(origin, top), HybridPathBuilder.SegmentType.CLAMPED_CUBIC,30.0, false);  // Forward
        pathManager.addLinearPath(List.of(top, origin), HybridPathBuilder.SegmentType.CLAMPED_CUBIC,30.0, true);   // Reverse

        telemetry.addLine("Ready to run forward and back path");
        telemetry.update();
        waitForStart();

        pathManager.startPath(0);

        boolean isWaiting = false;
        double waitStartTime = 0.0;
        double lastTime = getRuntime();

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            odometry.update();
            Pose2d currentPose = odometry.getPose();
            Pose2d holdPose = pathManager.getCurrentIndex() == 0 ? top : origin;

            pathManager.update();

            telemetry.addData("Progress", "%.2f", pathManager.getProgress());
            telemetry.addData("Pose Error", pathManager.getPoseError());
            telemetry.addData("Deviating", pathManager.isDeviating());
            telemetry.addData("Current Pose", currentPose);
            telemetry.addData("Target Pose", holdPose);
            telemetry.update();

            if (pathManager.isFinished()) {
                if (!isWaiting) {
                    isWaiting = true;
                    waitStartTime = currentTime;
                } else if (currentTime - waitStartTime < HOLD_DURATION) {
                    // Hold final pose with zero velocity
                    hardware.applyPower(
                            RobotPower.zero(), // basePower is zero; let correction drive everything
                            holdPose,
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
                } else {
                    isWaiting = false;
                    if (pathManager.getCurrentIndex() < pathManager.getPathCount() - 1) {
                        pathManager.startNextPath();
                    } else {
                        break;
                    }
                }
            }
        }

        hardware.stop();
        telemetry.addLine("Forward and back path complete");
        telemetry.update();
    }

    /** Normalizes angle to [-π, π] range. */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}