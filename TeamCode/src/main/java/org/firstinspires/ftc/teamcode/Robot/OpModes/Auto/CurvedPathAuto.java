package org.firstinspires.ftc.teamcode.Robot.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

import org.firstinspires.ftc.teamcode.Pathing.Follower.pathOperator;
import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing.QuinticHermiteSegment;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.Trajectory;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Waypoint;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Odometry;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

@Autonomous(name = "CurvedPathAuto")
public class CurvedPathAuto extends LinearOpMode {
    double maxVelocity = 30.0;     // inches per second
    double acceleration = 60.0;    // inches per second^2
    boolean reversed = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        OdometryHardware hardware = new OdometryHardware(hardwareMap);
        Odometry odometry = new Odometry(hardware);

        // Initialize pose
        Pose2d startPose = new Pose2d(0, 0, 0); // inches, radians
        odometry.reset(startPose);

        // Define curved path
        Waypoint start = new Waypoint(0, 0, 1, 0, 0, 0, 0);     // moving right
        Waypoint end   = new Waypoint(24, 24, 0, 1, 0, 0, 0);   // moving up

        QuinticHermiteSegment segment = new QuinticHermiteSegment(start, end);

        Trajectory trajectory = new Trajectory(
                List.of(segment),
                maxVelocity,
                acceleration,
                reversed
        );

        // Initialize path operator with OdometryHardware
        pathOperator operator = new pathOperator(hardware);
        operator.addPath(trajectory);

        telemetry.addData("Total Paths", operator.getTotalPaths());
        telemetry.addData("Current Index", operator.getCurrentIndex());
        telemetry.addLine("Ready to run curved path");
        telemetry.update();
        waitForStart();

        // Start path and apply initial movement
        Pose2d currentPose = odometry.getPose();
        operator.startPath(0, getRuntime(), currentPose);

        // Autonomous loop
        while (opModeIsActive() && !operator.isCurrentPathFinished(getRuntime())) {
            odometry.update(); // Update pose from encoders
            currentPose = odometry.getPose();
            operator.updateAndDrive(currentPose, getRuntime());

            telemetry.addData("Pose", currentPose);
            telemetry.addData("Follower Active", operator.getCurrentIndex() >= 0);
            telemetry.update();
        }

        // Stop motors
        hardware.stop();
        telemetry.addLine("Path complete");
        telemetry.update();
    }
}