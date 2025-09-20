package org.firstinspires.ftc.teamcode.Robot.Test.PathingTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Odometry;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

@Autonomous(name = "LocalizationPushTest")
public class LocalizationPushTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        OdometryHardware hardware = new OdometryHardware(hardwareMap);
        Odometry odometry = new Odometry(hardware);

        Pose2d startPose = new Pose2d(0, 0, 0);
        odometry.reset(startPose);

        telemetry.addLine("Push the robot to test localization");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            odometry.update();
            Pose2d pose = odometry.getPose();

            telemetry.addData("X (in)", pose.x);
            telemetry.addData("Y (in)", pose.y);
            telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading));
            telemetry.update();
        }
    }
}