package org.firstinspires.ftc.teamcode.Robot.Test.PathingTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Odometry;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

@Autonomous(name = "HeadingOffsetPushTest")
public class HeadingOffsetPushTest extends LinearOpMode {

    private static final double TARGET_ROTATION_DEGREES = 720.0; // Two full circles
    private static final double TARGET_ROTATION_RADIANS = Math.toRadians(TARGET_ROTATION_DEGREES);

    @Override
    public void runOpMode() {
        OdometryHardware hardware = new OdometryHardware(hardwareMap);
        Odometry odometry = new Odometry(hardware);

        Pose2d startPose = new Pose2d(0, 0, 0);
        odometry.reset(startPose);

        telemetry.addLine("Rotate robot manually through 2 full circles (720°)");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            odometry.update();
            Pose2d pose = odometry.getPose();

            double estimatedHeadingRad = pose.heading;
            double estimatedHeadingDeg = Math.toDegrees(estimatedHeadingRad);
            double suggestedOffset = estimatedHeadingRad == 0 ? 0 : TARGET_ROTATION_RADIANS / estimatedHeadingRad;

            telemetry.addData("Estimated Heading (deg)", "%.2f", estimatedHeadingDeg);
            telemetry.addData("Expected Rotation", "%.2f°", TARGET_ROTATION_DEGREES);
            telemetry.addData("Suggested HEADING_OFFSET", "%.4f", suggestedOffset);
            telemetry.addData("X (in)", "%.2f", pose.x);
            telemetry.addData("Y (in)", "%.2f", pose.y);
            telemetry.update();
        }
    }
}