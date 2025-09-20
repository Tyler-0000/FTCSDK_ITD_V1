package org.firstinspires.ftc.teamcode.Robot.Test.PathingTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Odometry;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

@Autonomous(name = "OdoPods")
public class OdoPods extends LinearOpMode {

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

            telemetry.addData("Left", odometry.ticksToInches(hardware.getLeftTicks()));
            telemetry.addData("Right", odometry.ticksToInches(hardware.getRightTicks()));
            telemetry.addData("Yaw", odometry.ticksToInches(hardware.getStrafeTicks()));
            telemetry.update();
        }
    }
}