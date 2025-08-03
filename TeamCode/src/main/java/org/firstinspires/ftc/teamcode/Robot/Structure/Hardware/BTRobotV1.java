package org.firstinspires.ftc.teamcode.Robot.Structure.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
public class BTRobotV1 {
    public int VL_Extension = 0;
    public int VL_Increment = 50;
    final public int MIN_VL_Height = 0;
    final public int MAX_VL_Height = 770;
    /*
    public double HL_Extension = 0;
    public double HL_Increment = 0.01;
    final public double MIN_HL_Distance = 0.0;
    final public double MAX_HL_Distance = 1.0;
     */

    public double HL_Extension = 0.3;
    public double HL_Increment = 0.05;
    final public double MIN_HL_Distance = 0.20;
    final public double MAX_HL_Distance = 0.90;

    public double I_Rotation = 0.0;
    public double I_Increment = 0.1;
    final public double MIN_I_Rotation = 0.0;
    final public double MAX_I_Rotation = 1.0;

    public double DW_Rotation = 0.63;
    public double DW_Increment = 0.01;
    final public double DW_MIN_Rotation = 0.63;
    final public double DW_MAX_Rotation = 1;

    public double DA_Rotation = 0.15;
    public double DA_Increment = 0.1;
    final public double DA_MIN_Rotation = 0.15;
    final public double DA_MAX_Rotation = 0.95;

    public String intakeColor;

    public double Poop_Pose = 0.0;

    private OpMode myOpMode;
    private ElapsedTime holdTimer = new ElapsedTime();

    private boolean showTelemetry = false;
    public BTRobotV1(OpMode opMode) {
        myOpMode = opMode;
    }
    //NOTE: when not talking about a lift L indicates Left and R indicates Right
    public DcMotor VLL, VLR, I;
    public Servo HLL, HLR, IL, IR, DW, DC, IP, ADAL, ADAR;
    public RevColorSensorV3 colorSensor;

    public TouchSensor touchSensor;
    public boolean touchSensorIsPressed = false;
    public double touchSensorValue;
    private double redValue;
    private double blueValue;
    private double greenValue;
    private double alphaValue; //light Intensity
    private static boolean isAuto = false;
    private double targetValue = 1000;

    public void setIsAuto(boolean tf) {
        isAuto = tf;
    }

    public void initialize(boolean showTelemetry) {
        //Vertical lift Motors
        if(isAuto) {
            VLL = setupAutoLiftMotor("VLL", DcMotor.Direction.FORWARD);
            VLR = setupAutoLiftMotor("VLR", DcMotor.Direction.REVERSE);
        } else {
            VLL = setupLiftMotor("VLL", DcMotor.Direction.FORWARD);
            VLR = setupLiftMotor("VLR", DcMotor.Direction.REVERSE);
        }

        //Horizontal lift Servo
        HLL = setupServo("HLL", Servo.Direction.REVERSE);
        HLR = setupServo("HLR", Servo.Direction.FORWARD);

        //Horizontal life Motors
        //HL = setupMotor("HL", DcMotorSimple.Direction.FORWARD);

        //Intake
        I = setupDriveMotor("I", DcMotor.Direction.REVERSE);

        //Intake Rotation Servos
        IL = setupServo("IL", Servo.Direction.FORWARD);
        IR = setupServo("IR", Servo.Direction.REVERSE);

        //Intake Servo Horn
        IP = setupServo("ISH", Servo.Direction.FORWARD);

        //Deposit Servo DW = "Deposit Wrist" DC = "Deposit Claw"
        DW = setupServo("DW", Servo.Direction.FORWARD);
        DC = setupServo("DC", Servo.Direction.FORWARD);

        //Deposit Arm ADAL = "Axon Deposit Arm Left" ADAR = "Axon Deposit Arm Right"
        ADAL = setupServo("ADAL", Servo.Direction.REVERSE);
        ADAR = setupServo("ADAR", Servo.Direction.FORWARD);

        colorSensor = (RevColorSensorV3) myOpMode.hardwareMap.get("colorSensor");

        colorSensor.enableLed(true);

        touchSensor = (TouchSensor) myOpMode.hardwareMap.get("touchSensor");

//        if(DA_Rotation < 0.15) {
//            ADAL.setPosition(0.15);
//            ADAR.setPosition(0.15);
//        }
    }

    //Setups the Drive Motor As Well As Setting Direction
    private DcMotor setupDriveMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return aMotor;
    }

    private DcMotor setupAutoLiftMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        aMotor.setTargetPosition(0);
        aMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return aMotor;
    }

    private DcMotor setupLiftMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        aMotor.setTargetPosition(0);
        aMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return aMotor;
    }

    //Setups the Servos As Well As Setting Direction
    private Servo setupServo(String deviceName, Servo.Direction direction) {
        Servo aServo = myOpMode.hardwareMap.get(Servo.class, deviceName);
        aServo.setDirection(direction);
        return aServo;
    }

    public void showTelemetry(boolean show) {
        showTelemetry = show;
    }
    public void TelemetryOutput() {
        myOpMode.telemetry.addData("Vertical_Lift",  VLR.getCurrentPosition());
        myOpMode.telemetry.addData("Horizontal_Lift",  HLL.getPosition());
        myOpMode.telemetry.addData("HL", HL_Extension);
        myOpMode.telemetry.addData("Intake_Rotation",  IR.getPosition());
        myOpMode.telemetry.addData("Deposit_Claw",  DC.getPosition());
        myOpMode.telemetry.addData("Deposit_Wrist",  DW.getPosition());
        myOpMode.telemetry.addData("Deposit_Arm", ADAR.getPosition());
        myOpMode.telemetry.addData("Red_Value", "%.2f", redValue);
        myOpMode.telemetry.addData("Green_Value", "%.2f", greenValue);
        myOpMode.telemetry.addData("Blue_Value", "%.2f", blueValue);
        myOpMode.telemetry.addData("Alpha_Value", "%.2f", alphaValue);
        myOpMode.telemetry.addData("Color", intakeColor);
        myOpMode.telemetry.addData("Intake Pow", I.getPower());
        myOpMode.telemetry.addData("touchSensorIsPressed", touchSensorIsPressed);
        myOpMode.telemetry.addData("touchSensorValue", "%.2f", touchSensorValue);
    }

    public void getColor(){
        redValue = colorSensor.red();
        greenValue = colorSensor.green();;
        blueValue = colorSensor.blue();
        alphaValue = colorSensor.alpha();

        if (blueValue >= greenValue && blueValue >= redValue) {
            intakeColor = "Blue";
        } else if (greenValue >= redValue) {
            intakeColor = "Yellow";
        } else {
            intakeColor = "Red";
        }
    }

    public void getTouchSensor() {
        touchSensorIsPressed = touchSensor.isPressed();
        touchSensorValue = touchSensor.getValue();
    }

    public void resetSlides() {
        if(!touchSensorIsPressed) {
            resetVertical_Lift(true);
        } else{
            VLL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            VLL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            VLL.setTargetPosition(0);
            VLL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            VLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            VLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            VLR.setTargetPosition(0);
            VLR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            VL_Extension = 0;
        }
    }

    public void resetVertical_Lift(boolean t){
        if(t){
            VL_Extension -= VL_Increment;
            Setup_Vertical_Lift(VL_Extension, 1.0);
        }
    }

    public void Vertical_Lift(boolean t) {
        if(t){
            VL_Extension += VL_Increment;
            VL_Extension = Math.max(MIN_VL_Height, Math.min(MAX_VL_Height, VL_Extension));
            Setup_Vertical_Lift(VL_Extension, 1.0);
        } else {
            VL_Extension -= VL_Increment * 5;
            VL_Extension = Math.max(MIN_VL_Height, Math.min(MAX_VL_Height, VL_Extension));
            Setup_Vertical_Lift(VL_Extension, 1.0);
        }
    }

    public void Horizontal_Lift(boolean t) {
        if(t){
            HL_Extension += HL_Increment;
            HL_Extension = Math.max(MIN_HL_Distance, Math.min(MAX_HL_Distance, HL_Extension));
            Setup_Horizontal_Lift(HL_Extension);
        } else {
            HL_Extension -= HL_Increment;
            HL_Extension = Math.max(MIN_HL_Distance, Math.min(MAX_HL_Distance, HL_Extension));
            Setup_Horizontal_Lift(HL_Extension);
        }
    }

    /*public void Horizontal_Lift(boolean t) {
        if(t){
            HL_Extension += HL_Increment;
            HL_Extension = Math.max(MIN_HL_Distance, Math.min(MAX_HL_Distance, HL_Extension));
            Setup_Horizontal_Lift(HL_Extension);
        } else {
            HL_Extension -= HL_Increment;
            HL_Extension = Math.max(MIN_HL_Distance, Math.min(MAX_HL_Distance, HL_Extension));
            Setup_Horizontal_Lift(HL_Extension);
        }
    }*/

    public void Intake_Pose(boolean t) {
        if(t){
            I_Rotation += I_Increment;
            I_Rotation = Math.max(MIN_I_Rotation, Math.min(MAX_I_Rotation, I_Rotation));
            Setup_Intake_Pose(I_Rotation);
        } else {
            I_Rotation -= I_Increment;
            I_Rotation = Math.max(MIN_I_Rotation, Math.min(MAX_I_Rotation, I_Rotation));
            Setup_Intake_Pose(I_Rotation);
        }
    }

    public void Deposit_Wrist(boolean t) {
        if(t){
            DW_Rotation += DW_Increment;
            DW_Rotation = Math.max(DW_MIN_Rotation, Math.min(DW_MAX_Rotation, DW_Rotation));
            Setup_Deposit_Wrist(DW_Rotation);
        } else {
            DW_Rotation -= DW_Increment;
            DW_Rotation = Math.max(DW_MIN_Rotation, Math.min(DW_MAX_Rotation, DW_Rotation));
            Setup_Deposit_Wrist(DW_Rotation);
        }
    }

    public void Deposit_Arm(boolean t){
        if(t){
            DA_Rotation += DA_Increment;
            DA_Rotation = Math.max(DA_MIN_Rotation, Math.min(DA_MAX_Rotation, DA_Rotation));
            Setup_Deposit_Arm(DA_Rotation);
        } else {
            DA_Rotation -= DA_Increment;
            DA_Rotation = Math.max(DA_MIN_Rotation, Math.min(DA_MAX_Rotation, DA_Rotation));
            Setup_Deposit_Arm(DA_Rotation);
        }
    }

    public void Setup_Vertical_Lift(int EXT, double pow) {
        VL_Extension = EXT;
        VLL.setTargetPosition(VL_Extension);
        VLR.setTargetPosition(VL_Extension);
        VLL.setPower(pow);
        VLR.setPower(pow);
    }

    public void Setup_Horizontal_Lift(double EXT) {
        HL_Extension = EXT;
        HLL.setPosition(HL_Extension);
        HLR.setPosition(HL_Extension);
    }

    /*public void Setup_Horizontal_Lift(double EXT) {
        HL_Extension = EXT;
        HLL.setPosition(EXT);
        HLR.setPosition(EXT);
    }*/

    public void Setup_Intake_Pose(double Rot) {
        I_Rotation = Rot;
        IL.setPosition(I_Rotation);
        IR.setPosition(I_Rotation);
    }

    public void Setup_Intake_Pose_RTP(boolean t) {
        if(t) {
            IL.setPosition(0.42);
            IR.setPosition(0.42);
        } else{
            IL.setPosition(0.0);
            IR.setPosition(0.0);
        }
    }

    public void Setup_Deposit_Claw(boolean t) {
        if(t){
            DC.setPosition(0.3);
        } else {
            DC.setPosition(0.0);
        }
    }

    public void Setup_Deposit_Wrist(double Rot) {
        DW_Rotation = Rot;
        DW.setPosition(DW_Rotation);
    }

    public void Intake(double pow) {
        I.setPower(pow);
    }

    public void Intake_Poop(boolean t) {
        if(t){
            IP.setPosition(0.0);
        } else {
            IP.setPosition(0.5);
        }
    }

    public void Setup_Deposit_Arm(double Rot){
        DA_Rotation = Rot;
        ADAR.setPosition(DA_Rotation);
        ADAL.setPosition(DA_Rotation);
    }

    public void SpecimenScore(){
        Setup_Deposit_Claw(false);
        Setup_Deposit_Arm(0.22);
        Setup_Deposit_Wrist(0.9);
        Setup_Vertical_Lift(475, 1.0);
    }

    public void verticalSlideUp(){
        Setup_Vertical_Lift(800, 1.0);
    }

    public void HighBasketScore(){
        Setup_Intake_Pose(0.2);
        Setup_Deposit_Arm(0.6);
        Setup_Deposit_Wrist(0.75);
        Setup_Vertical_Lift(770, 1.0);
    }

    public void SpecimenGrab(){
        Setup_Deposit_Claw(true);
        Setup_Deposit_Arm(0.95);
        Setup_Deposit_Wrist(0.63);
        Setup_Vertical_Lift(80, 1.0);
    }

    public void TransferSample(){
        Setup_Deposit_Arm(0.15);
        Setup_Deposit_Wrist(0.0);
        Setup_Vertical_Lift(0, 0.7);
    }
}
