package org.firstinspires.ftc.teamcode.Robot.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pathing.Follower.Abstracted.PathManager;
import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathBuilder;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.Trajectory;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation.PIDCorrectionStrategy;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation.SimpleScalar;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation.WeightedBlender;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Odometry;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

import java.util.List;

@Autonomous(name = "FullPathingDebugTest")
public class FullPathingDebugTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Core hardware and odometry
        OdometryHardware hardware = new OdometryHardware(hardwareMap);
        Odometry odometry = new Odometry(hardware);

        Trajectory testTrajectory = new Trajectory(
                HybridPathBuilder.buildCurvePath(
                        List.of(
                                new Pose2d(0, 0, 0),
                                new Pose2d(12, 12, Math.PI / 4),
                                new Pose2d(24, 0, 0)
                        )
                ),
                0.0,    // max velocity
                false   // not reversed
        );

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

        pathManager.addTrajectory(testTrajectory);

        telemetry.addLine("Ready to debug pathing stack");
        telemetry.update();
        waitForStart();

        pathManager.startPath(0);
        double lastTime = getRuntime();

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            odometry.update();

            //Waypoints
            HybridPathBuilder.logWaypoints(testTrajectory.getWaypoints(), telemetry);

            // Pose error
            Pose2d error = pathManager.getPoseError();
            telemetry.addData("Forward Error (x)", error.x);
            telemetry.addData("Strafe Error (y)", error.y);
            telemetry.addData("Heading Error (rad)", error.heading);

            // Velocity diagnostics from follower
            if (pathManager.getFollower() != null) {
                telemetry.addData("Target Pose", pathManager.getFollower().getTargetPose());
                telemetry.addData("Current Velocity", pathManager.getFollower().getCurrentVelocity());
                telemetry.addData("Target Velocity", pathManager.getFollower().getTargetVelocity());
            }

            // Deviation flag
            telemetry.addData("Is Deviating", pathManager.isDeviating());

            // Motor power
            hardware.logMotorPower(telemetry);

            // Encoder snapshot
            telemetry.addData("Encoders", hardware.getEncoderSnapshot());

            // Stall detection
            if (hardware.isStalled(5.0, 0.2)) {
                telemetry.addLine("Stall detected — attempting recovery");
                hardware.attemptStallRecovery();
            }

            pathManager.update();
            telemetry.update();
        }

        hardware.stop();
        telemetry.addLine("Debug complete");
        telemetry.update();
    }
}
