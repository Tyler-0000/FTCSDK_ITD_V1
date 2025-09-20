package org.firstinspires.ftc.teamcode.Robot.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Odometry;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

@Autonomous(name = "Odometry Diagnostic")
public class OdometryDiagnosticOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        OdometryHardware hardware = new OdometryHardware(hardwareMap);
        Odometry odometry = new Odometry(hardware);

        telemetry.addLine("Odometry Diagnostic Ready");
        telemetry.update();
        waitForStart();

        double lastTime = getRuntime();

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            odometry.update();
            Pose2d pose = odometry.getPose();

            telemetry.addData("Δt", "%.3f", dt);
            telemetry.addData("Pose X (field)", "%.2f", pose.x);
            telemetry.addData("Pose Y (field)", "%.2f", pose.y);
            telemetry.addData("Heading (rad)", "%.2f", pose.heading);
            telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.heading));
            telemetry.update();
        }
    }
}