package org.firstinspires.ftc.teamcode.Robot.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

import org.firstinspires.ftc.teamcode.Pathing.Follower.pathOperator;
import org.firstinspires.ftc.teamcode.Pathing.Hardware.DriveBase;
import org.firstinspires.ftc.teamcode.Pathing.Hardware.OdometryHardware;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing.QuinticHermiteSegment;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.Trajectory;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;
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
        DriveBase driveBase = new DriveBase(hardwareMap);
        OdometryHardware odoHardware = new OdometryHardware(hardwareMap);
        Odometry odometry = new Odometry(odoHardware);

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

        // Initialize path operator
        pathOperator operator = new pathOperator();
        operator.addPath(trajectory);
        operator.startPath(0, getRuntime());

        telemetry.addLine("Ready to run curved path");
        telemetry.update();
        waitForStart();

        // Autonomous loop
        while (opModeIsActive() && !operator.isCurrentPathFinished(getRuntime())) {
            odometry.update(); // Update pose from encoders

            Pose2d currentPose = odometry.getPose();
            RobotPower power = operator.update(currentPose, getRuntime());
            driveBase.applyPower(power);

            telemetry.addData("Pose", currentPose);
            telemetry.addData("Power", power);
            telemetry.update();
        }

        // Stop motors
        driveBase.stop();
        telemetry.addLine("Path complete");
        telemetry.update();
    }
}