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
    private final Pose pickup1Pose = new Pose(23, 124, Math.toRadians(355));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(21, 129.5, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(20, 132, Math.toRadians(20));

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

    }
    public void intakeBack(){

//        robot.Intake(0);
//        robot.Setup_Deposit_Claw(false);
//        robot.wristBack();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                realState = 0;
                telemetry.addLine("case0");
                telemetry.update();
//                robot.Setup_Intake_Pose_RTP(true);
//                robot.wristOut();
//                robot.Setup_Horizontal_Lift(0.0);
//                robot.Setup_Intake_Pose(0);
                raiseArm();
                if(pathTimer.getElapsedTimeSeconds()>0.5){
                    follower.followPath(scorePreload);
                    setPathState(1);
                }
                break;
//            case 101:
//                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.102) {
////                    robot.Setup_Deposit_Claw(false);
//                    raiseArm();
//                    if (pathTimer.getElapsedTimeSeconds()>3.102){
//                        setPathState(1);
//                    }
//                }
//                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    realState = 1;
                    highScoreWithDelay(0.0);
                    robot.Setup_Intake_Pose(0);
                    if(pathTimer.getElapsedTimeSeconds()>2.5){
                        follower.followPath(grabPickup1,true);
                        setPathState(201);
                    }
                }
                break;
            case 201:
                if(!follower.isBusy()){
                    intakeOut();
                    setPathState(202);
                }
                break;
            case 202:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>2){
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy() &&  pathTimer.getElapsedTimeSeconds()>0.5) {
                    realState = 2;
                    intakeBack();
                    setPathState(205);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                }
                break;
            case 205:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>1.5){
                    //robot.Intake(0);
                    follower.followPath(scorePickup1,true);
                    setPathState(301);
                }
                break;
            case 301:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.001) {
                    robot.Setup_Deposit_Claw(false);
                    realState = 301;
                    setPathState(3011);
                }
                break;
            case 3011:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.21){
                    realState = 3011;
                    robot.Setup_Intake_Pose(0.2);
                    setPathState(302);
                }
                break;
            case 302:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.302) {
//                    robot.Setup_Deposit_Claw(false);
                    raiseArm();
                    realState = 302;
                    setPathState(303);
                }
                break;
            case 303:
                if (pathTimer.getElapsedTimeSeconds()>1.102){
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    realState = 3;
                    highScoreWithDelay(0.0);
                    if(pathTimer.getElapsedTimeSeconds()>3){
                        follower.followPath(grabPickup2,true);
                        setPathState(401);
                    }
                }
                break;
            case 401:
                if(!follower.isBusy() &&  pathTimer.getElapsedTimeSeconds()>0.6){
                    intakeOut();
                    setPathState(402);
                }
                break;
            case 402:
                if(!follower.isBusy() &&pathTimer.getElapsedTimeSeconds()>2){
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy() ) {
                    realState = 4;
                    robot.Intake_Poop(true);
                    robot.Setup_Intake_Pose_RTP(true);
                    intakeBack();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if(pathTimer.getElapsedTimeSeconds()>2.5){
                        //robot.Intake(0);
                        follower.followPath(scorePickup2,true);
                        setPathState(591);
                    }
                }
                break;
            case 591:
                if(!follower.isBusy() ) {
//                    robot.Setup_Deposit_Claw(false);
                    robot.Setup_Intake_Pose_RTP(true);
                    realState = 591;
                    if (pathTimer.getElapsedTimeSeconds()>0.15){
//                        robot.Setup_Intake_Pose_RTP(false);
                        setPathState(501);
                    }
                }
                break;
            case 501:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.001) {
//                    robot.Setup_Deposit_Claw(false);
                    robot.Setup_Deposit_Claw(false);
                    realState = 501;
                    if (pathTimer.getElapsedTimeSeconds()>0.15){
                        robot.Setup_Intake_Pose(0.2);
//                        robot.Setup_Intake_Pose_RTP(false);
                        setPathState(502);
                    }
                }
                break;
            case 502:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.302) {
//                    robot.Setup_Deposit_Claw(false);
                    raiseArm();
                    realState = 302;
                    setPathState(505);

                }
                break;
            case 505:
                if (!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>1){
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    realState = 3;
                    highScoreWithDelay(0.0);
                    if(pathTimer.getElapsedTimeSeconds()>3){
                        follower.followPath(grabPickup3,true);
                        setPathState(601);
                    }
                }
                break;
            case 601: //
                if(!follower.isBusy() ) {
                    intakeOut();
                        setPathState(602);

                }
                break;
            case 602:
                if(!follower.isBusy() &&pathTimer.getElapsedTimeSeconds()>2){
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    realState = 6;
                    intakeBack();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if(pathTimer.getElapsedTimeSeconds()>1.4){
                        //robot.Intake(0);
                        intakeBack();
                        follower.followPath(scorePickup3,true);
                        setPathState(791);
                    }
                }
                break;
            case 791:   //lift intaker for 3rd sample grip
                if(!follower.isBusy() ) {
                    robot.Setup_Intake_Pose_RTP(true);
                    realState = 791;
                    if (pathTimer.getElapsedTimeSeconds()>0.2){
                        setPathState(701);
                    }
                }
                break;
            case 701:   //grip 3rd sample
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.001) {
                    robot.Setup_Deposit_Claw(false);
                    realState = 701;
                    if (pathTimer.getElapsedTimeSeconds()>0.16){
                        robot.Setup_Intake_Pose(0.2);
                        setPathState(702);
                    }
                }
                break;
            case 702:   //raise arm for 3rd sample
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.35) {
//                    robot.Setup_Deposit_Claw(false);
                    raiseArm();
                    realState = 702;
                        setPathState(708);
                }
                break;
            case 708:
                if (!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>1) {
                    setPathState(7);
                }
                break;
            case 7:     //score the 3rd sample
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    realState = 3;
                    highScoreWithDelay(0.0);
                    if(pathTimer.getElapsedTimeSeconds()>3){
                        robot.Setup_Vertical_Lift(0, 1.0);
                        follower.followPath(park,true);
                        setPathState(8);
                    }
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    realState = 8;
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
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
