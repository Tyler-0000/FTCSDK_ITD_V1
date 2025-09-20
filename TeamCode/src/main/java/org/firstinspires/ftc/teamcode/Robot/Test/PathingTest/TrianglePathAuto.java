package org.firstinspires.ftc.teamcode.Robot.Test.PathingTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pathing.Follower.Abstracted.PathManager;
import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathBuilder;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.*;
import org.firstinspires.ftc.teamcode.Pathing.Optimization.Efficiency.EfficiencyModel;
import org.firstinspires.ftc.teamcode.Pathing.Optimization.Speed.SpeedOptimizer;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

import java.util.List;

@Autonomous(name = "TrianglePathAuto")
public class TrianglePathAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        OdometryHardware hardware = new OdometryHardware(hardwareMap);

        PathManager pathManager = new PathManager(
                hardware,
                telemetry,
                (t, c, d) -> RobotPower.zero(), // CorrectionStrategy
                (r, p, d) -> r,                 // CorrectionScaler
                (b, c) -> b,                    // CorrectionBlender
                (c, t, r, s, f) -> {},          // DiagnosticLogger
                (p, pose, v) -> p,              // EfficiencyModel
                (p, pose, v) -> p               // SpeedOptimizer
        );

        Pose2d A = new Pose2d(0, 0, 0);
        Pose2d B = new Pose2d(24, 0, 0);
        Pose2d C = new Pose2d(12, 20, 0);

        pathManager.addLinearPath(List.of(A, B), HybridPathBuilder.SegmentType.BEZIER,30.0, false);
        pathManager.addLinearPath(List.of(B, C), HybridPathBuilder.SegmentType.BEZIER,30.0, false);
        pathManager.addLinearPath(List.of(C, A), HybridPathBuilder.SegmentType.BEZIER,30.0, false);

        telemetry.addLine("Ready to run triangle path");
        telemetry.update();
        waitForStart();

        pathManager.startPath(0);

        while (opModeIsActive()) {
            pathManager.update();

            telemetry.addData("Progress", pathManager.getProgress());
            telemetry.addData("Pose Error", pathManager.getPoseError());
            telemetry.addData("Deviating", pathManager.isDeviating());
            telemetry.update();

            if (pathManager.isFinished()) {
                if (pathManager.getCurrentIndex() < pathManager.getPathCount() - 1) {
                    pathManager.startNextPath();
                } else {
                    break;
                }
            }
        }

        hardware.stop();
        telemetry.addLine("Triangle path complete");
        telemetry.update();
    }
}