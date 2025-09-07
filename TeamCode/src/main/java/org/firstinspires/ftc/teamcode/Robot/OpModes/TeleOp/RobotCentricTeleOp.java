package org.firstinspires.ftc.teamcode.Robot.OpModes.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Structure.Library.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.Robot.Structure.Hardware.BTRobotV1;

@TeleOp(name = "Robot-Centric Teleop", group = "Linear OpMode")
public class RobotCentricTeleOp extends LinearOpMode {
    BTRobotV1 robot = new BTRobotV1(this);

    private ElapsedTime runtime = new ElapsedTime();
    private Follower follower;
    private Path scoreBasket;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private final Pose startPose = PoseStorage.CurrentPose;
    public String Color_Alliance = null;
    private final Pose scorePose = new Pose(18, 130, Math.toRadians(315));

    private double LSY = 1.0, LSX = 1.0, RSX = 1.0;

    private boolean isAutoDriving = false;

    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();

    private String Mode = "Regular_Mode";
    @Override
    public void runOpMode() {
        robot.setIsAuto(false);
        robot.initialize(true);
        while (opModeInInit()) {
            telemetry.addData(">", "Touch Play to Drive");

            follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
            follower.setStartingPose(startPose);

            pathTimer = new Timer();
            opmodeTimer = new Timer();
            opmodeTimer.resetTimer();

            follower.startTeleopDrive();

//            robot.colorSensor.enableLed(true);
            robot.getColor();
        }
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Setting all target positions to zero so it doesn't jitter
        waitForStart();
        runtime.reset();

        while(opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            robot.getColor();

            if(currentGamepad1.x && !previousGamepad1.x){
                Color_Alliance = "Blue";
            } else if (currentGamepad1.b && !previousGamepad1.b) {
                Color_Alliance = "Red";
            }

            robot.Intake_Poop(robot.intakeColor.equals(Color_Alliance) || robot.intakeColor.equals("Yellow"));
            
            if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                isAutoDriving = true;
                follower.breakFollowing();
                scoreBasket = new Path(new BezierLine(new Point(follower.getPose()), new Point(scorePose)));
                scoreBasket.setConstantHeadingInterpolation(315);
                follower.followPath(scoreBasket);
            }

            if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                isAutoDriving = false;
                follower.startTeleopDrive();
            }

            if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                follower.setPose(new Pose(18, 130, 315));
            }

            if(currentGamepad1.a && !previousGamepad1.a) {
                LSY = 0.5;
                LSX = 0.5;
                RSX = 0.3;
                Mode = "Baby_Mode";
            } else if (currentGamepad1.y && !previousGamepad1.y) {
                LSY = 1.0;
                LSX = 1.0;
                RSX = 1.0;
                Mode = "Regular_Mode";
            }



            if(!isAutoDriving) {
                follower.setTeleOpMovementVectors(
                        -gamepad1.left_stick_y * LSY,
                        -gamepad1.left_stick_x * LSX,
                        -gamepad1.right_stick_x * RSX,
                        true);
                follower.update();
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Mode", Mode);
            telemetry.addData("Forward", LSY);
            telemetry.addData("Strafe", LSX);
            telemetry.addData("Turn", RSX);
            telemetry.addData("Color_Alliance", Color_Alliance);
            /* Telemetry Outputs of our Follower */
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("follower", follower.isBusy());
            //telemetry.addData("Intake sampleColor", Intake.sampleColor);
            robot.TelemetryOutput();
            /* Update Telemetry to the Driver Hub */
            telemetry.update();
        }
    }
}