package org.firstinspires.ftc.teamcode.Robot.OpModes.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Structure.Hardware.BTRobotV1;
import org.firstinspires.ftc.teamcode.Robot.Structure.Library.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "Sample_Auto", group = "OpMode")
public class Samples extends OpMode {
    BTRobotV1 robot = new BTRobotV1(this);
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState, commandState;
    private int realState;

    public String Color_Alliance = null;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
//pre 12:18    private final Pose scorePose = new Pose(14, 126, Math.toRadians(315));
    //pre 12:18    private final Pose scorePose = new Pose(17, 129, Math.toRadians(315));

    private final Pose scorePose = new Pose(16, 130, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(18, 125, Math.toRadians(355));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(18, 132, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(20, 130, Math.toRadians(25));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));
    /**Intake from Submersible */
    private final Pose pickupSubPose = new Pose(65, 95, Math.toRadians(270));

    /** Used as the control point for the BeizerCurve for SubPose*/
    private final Pose pickupSubControlPose = new Pose(55, 130, Math.toRadians(270));

    /** Park Pose for our robot, after we do all of the scoring. */

    /** Used as the control point for the BeizerCurve for parkPose*/
    private final Pose parkControlPose = new Pose(63, 110, Math.toRadians(90));
    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    //    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, grabSubPose1, scoreSubPose1, grabSubPose2, scoreSubPose2;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        grabSubPose1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickupSubControlPose), new Point(pickupSubPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupSubPose.getHeading())
                .build();

        scoreSubPose1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupSubPose), new Point(pickupSubControlPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickupSubPose.getHeading(), scorePose.getHeading())
                .build();

        grabSubPose2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickupSubControlPose), new Point(pickupSubPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupSubPose.getHeading())
                .build();

        scoreSubPose2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupSubPose), new Point(pickupSubControlPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickupSubPose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */

    public void sleepMethod(double timeInSeconds) {
        long temp = (long) (timeInSeconds * 1000);
        try {
            //Pause the current thread for x
            Thread.sleep(temp);
        } catch (InterruptedException e) {
            // Handle the InterruptedException, which occurs if another thread interrupts this one
            System.err.println("Thread interrupted during sleep: " + e.getMessage());
            // Re-interrupt the current thread to indicate that it was interrupted
            Thread.currentThread().interrupt();
        }
    }
    public void raiseArm(){
        robot.Setup_Deposit_Arm(0.5);
        robot.Setup_Deposit_Claw(false);
//        robot.Setup_Deposit_Claw(false);
    }
    public void transferSample(){
        robot.Setup_Deposit_Claw(false);
    }
    public void highScoreWithDelay(double delay){
        if (pathTimer.getElapsedTimeSeconds()<1.5+delay){
            robot.verticalSlideUp();
            robot.Setup_Deposit_Claw(false);
            robot.Setup_Deposit_Arm(0.50);
            robot.Deposit_Wrist(false);

        }else if(pathTimer.getElapsedTimeSeconds()<1.7+delay){
            robot.Deposit_Wrist(true);
            robot.Setup_Deposit_Arm(0.60);

        }
        else if (pathTimer.getElapsedTimeSeconds()<2.0+delay){
            robot.Setup_Deposit_Claw(true);
        }
        else if (pathTimer.getElapsedTimeSeconds()<2.4+delay){
            robot.Setup_Deposit_Arm(0.15);
            robot.Deposit_Wrist(false);
        }
        else if (pathTimer.getElapsedTimeSeconds()<2.9+delay){
            robot.TransferSample();
            intakeBack();
        }
    }
    public void intakeOut(){
        //not timing it, because very likely it will be followed by a pedro moving
        robot.Intake(-0.5);
        robot.Setup_Intake_Pose_RTP(false);
        robot.Setup_Horizontal_Lift(1.0);
    }
    public void intakeBack(){
        robot.Setup_Intake_Pose_RTP(true);
        robot.Setup_Horizontal_Lift(0.0);
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                realState = 0;
                robot.Setup_Intake_Pose_RTP(false);
                raiseArm();
                setPathState(1);
                break;
            case 1:
                if(pathTimer.getElapsedTimeSeconds()>0.7){
                    realState = 1;
                    follower.followPath(scorePreload);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    realState = 11;
                    highScoreWithDelay(0.0);
                    robot.Setup_Intake_Pose(0);
                    robot.Setup_Intake_Pose_RTP(false);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy() &&  pathTimer.getElapsedTimeSeconds()>2) {
                    realState = 2;
                    follower.followPath(grabPickup1,true);
                    setPathState(21);
                }
                break;
            case 21:
                if(!follower.isBusy()){
                    realState = 21;
                    intakeOut();
                    setPathState(22);
                }
                break;
            case 22:
                if(pathTimer.getElapsedTimeSeconds()>1.2){
                    realState = 22;
                    robot.Setup_Intake_Pose_RTP(true);
                    robot.Intake(0);
                    setPathState(23);
                }
                break;
            case 23:
                if(!follower.isBusy() &&  pathTimer.getElapsedTimeSeconds()>0.3) {
                    realState = 23;
                    intakeBack();
                    setPathState(3);
                }
                break;
            case 3:
                if(pathTimer.getElapsedTimeSeconds()>1){
                    realState = 3;
                    follower.followPath(scorePickup1,true);
                    setPathState(31);
                }
                break;
            case 31:
                if(!follower.isBusy()) {
                    realState = 31;
                    robot.Setup_Deposit_Claw(false);
                    setPathState(32);
                }
                break;
            case 32:
                if (pathTimer.getElapsedTimeSeconds()>0.2){
                    realState = 32;
                    robot.Setup_Intake_Pose(0.2);
                    setPathState(33);
                }
                break;
            case 33:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.05) {
                    realState = 33;
                    raiseArm();
                    setPathState(34);
                }
                break;
            case 34:
                if (pathTimer.getElapsedTimeSeconds()>2){
                    realState = 34;
                    setPathState(35);
                }
                break;
            case 35:
                if(!follower.isBusy()) {
                    realState = 35;
                    highScoreWithDelay(0.0);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy() &&  pathTimer.getElapsedTimeSeconds()>2) {
                    realState = 4;
                    follower.followPath(grabPickup2,true);
                    setPathState(41);
                }
                break;
            case 41:
                if(!follower.isBusy()){
                    realState = 41;
                    intakeOut();
                    setPathState(42);
                }
                break;
            case 42:
                if(pathTimer.getElapsedTimeSeconds()>1){
                    realState = 42;
                    robot.Setup_Intake_Pose_RTP(true);
                    robot.Intake(0);
                    setPathState(43);
                }
                break;
            case 43:
                if(!follower.isBusy() &&  pathTimer.getElapsedTimeSeconds()>0.5) {
                    realState = 43;
                    intakeBack();
                    setPathState(5);
                }
                break;
            case 5:
                if(pathTimer.getElapsedTimeSeconds()>1){
                    realState = 5;
                    follower.followPath(scorePickup2,true);
                    setPathState(51);
                }
                break;
            case 51:
                if(!follower.isBusy()) {
                    realState = 51;
                    robot.Setup_Deposit_Claw(false);
                    setPathState(52);
                }
                break;
            case 52:
                if (pathTimer.getElapsedTimeSeconds()>0.2){
                    realState = 52;
                    robot.Setup_Intake_Pose(0.2);
                    setPathState(53);
                }
                break;
            case 53:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.1) {
                    realState = 53;
                    raiseArm();
                    setPathState(54);
                }
                break;
            case 54:
                if (pathTimer.getElapsedTimeSeconds()>3){
                    realState = 54;
                    setPathState(55);
                }
                break;
            case 55:
                if(!follower.isBusy()) {
                    realState = 55;
                    highScoreWithDelay(0.0);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy() &&  pathTimer.getElapsedTimeSeconds()>2) {
                    realState = 6;
                    follower.followPath(grabPickup3,true);
                    setPathState(61);
                }
                break;
            case 61:
                if(!follower.isBusy()){
                    realState = 61;
                    intakeOut();
                    setPathState(62);
                }
                break;
            case 62:
                if(pathTimer.getElapsedTimeSeconds()>1){
                    realState = 62;
                    robot.Setup_Intake_Pose_RTP(true);
                    robot.Intake(0);
                    setPathState(63);
                }
                break;
            case 63:
                if(!follower.isBusy() &&  pathTimer.getElapsedTimeSeconds()>0.5) {
                    realState = 63;
                    intakeBack();
                    setPathState(7);
                }
                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds()>1){
                    realState = 7;
                    follower.followPath(scorePickup3,true);
                    setPathState(71);
                }
                break;
            case 71:
                if(!follower.isBusy()) {
                    realState = 71;
                    robot.Setup_Deposit_Claw(false);
                    setPathState(72);
                }
                break;
            case 72:
                if (pathTimer.getElapsedTimeSeconds()>0.2){
                    realState = 72;
                    robot.Setup_Intake_Pose(0.2);
                    setPathState(73);
                }
                break;
            case 73:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.1) {
                    realState = 73;
                    raiseArm();
                    setPathState(74);
                }
                break;
            case 74:
                if (pathTimer.getElapsedTimeSeconds()>3){
                    realState = 74;
                    setPathState(75);
                }
                break;
            case 75:
                if(!follower.isBusy()) {
                    realState = 75;
                    highScoreWithDelay(0.0);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    realState = 8;
                    highScoreWithDelay(0.0);
                    if(pathTimer.getElapsedTimeSeconds()>3){
                        robot.Setup_Vertical_Lift(0, 1.0);
                        follower.followPath(park,true);
                        intakeOut();
                        setPathState(9);
                    }
                }
                break;
            case 9:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    realState = 8;
                    setPathState(-1);
                }
                break;
        }

    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        actionTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
        robot.setIsAuto(true);
        robot.initialize(true);
        robot.colorSensor.enableLed(true);
        robot.getColor();
        Color_Alliance = "Blue";
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        PoseStorage.CurrentPose = follower.getPose();
    }

}