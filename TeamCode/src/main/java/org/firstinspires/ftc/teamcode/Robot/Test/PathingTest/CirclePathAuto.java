package org.firstinspires.ftc.teamcode.Robot.Test.PathingTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pathing.Follower.Abstracted.PathManager;
import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.*;
import org.firstinspires.ftc.teamcode.Pathing.Optimization.Efficiency.EfficiencyModel;
import org.firstinspires.ftc.teamcode.Pathing.Optimization.Speed.SpeedOptimizer;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "CirclePathAuto")
public class CirclePathAuto extends LinearOpMode {

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

        // Generate circular arc around center (12, 0) with radius 12
        List<Pose2d> circlePoints = new ArrayList<>();
        double radius = 12.0;
        int samples = 36; // 10° increments
        for (int i = 0; i <= samples; i++) {
            double angle = 2 * Math.PI * i / samples;
            double x = 12 + radius * Math.cos(angle);
            double y = radius * Math.sin(angle);
            double heading = angle + Math.PI / 2; // tangent to circle
            circlePoints.add(new Pose2d(x, y, heading));
        }

        pathManager.addCurvePath(circlePoints, 30.0, false);

        telemetry.addLine("Ready to run circular path");
        telemetry.update();
        waitForStart();

        pathManager.startPath(0);

        while (opModeIsActive()) {
            pathManager.update();

            telemetry.addData("Progress", pathManager.getProgress());
            telemetry.addData("Pose Error", pathManager.getPoseError());
            telemetry.addData("Curvature", pathManager.getCurvatureSamples(1));
            telemetry.addData("Deviating", pathManager.isDeviating());
            telemetry.update();

            if (pathManager.isFinished()) break;
        }

        hardware.stop();
        telemetry.addLine("Circle path complete");
        telemetry.update();
    }
}