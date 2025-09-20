package org.firstinspires.ftc.teamcode.Robot.Test.PathingTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Odometry;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

@Autonomous(name = "XOffsetPushTest")
public class XOffsetPushTest extends LinearOpMode {

    private static final double TILE_LENGTH_INCHES = 24.0; // 2 feet per tile
    private static final double TARGET_DISTANCE_INCHES = TILE_LENGTH_INCHES * 4; // 5 tiles = 120 inches

    @Override
    public void runOpMode() {
        OdometryHardware hardware = new OdometryHardware(hardwareMap);
        Odometry odometry = new Odometry(hardware);

        Pose2d startPose = new Pose2d(0, 0, 0);
        odometry.reset(startPose);

        telemetry.addLine("Push robot forward exactly 5 tiles (120 inches)");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            odometry.update();
            Pose2d pose = odometry.getPose();

            double estimatedX = pose.x;
            double suggestedOffset = estimatedX == 0 ? 0 : TARGET_DISTANCE_INCHES / estimatedX;

            telemetry.addData("Estimated X (in)", "%.2f", estimatedX);
            telemetry.addData("Expected Distance", "%.2f", TARGET_DISTANCE_INCHES);
            telemetry.addData("Suggested X_OFFSET", "%.4f", suggestedOffset);
            telemetry.addData("Y (in)", "%.2f", pose.y);
            telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.heading));
            telemetry.update();
        }
    }
}