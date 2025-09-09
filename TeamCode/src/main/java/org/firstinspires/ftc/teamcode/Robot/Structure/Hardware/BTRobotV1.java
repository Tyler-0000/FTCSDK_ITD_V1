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
    final public int MAX_VL_Height = 760;
    /*
    public double HL_Extension = 0;
    public double HL_Increment = 0.01;
    final public double MIN_HL_Distance = 0.0;
    final public double MAX_HL_Distance = 1.0;
     */

    public double HL_Extension = 0.3;
    public double HL_Increment = 0.05;
    final public double MIN_HL_Distance = 0.3;
    final public double MAX_HL_Distance = 1.0;

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

    }

    //Setups the Drive Motor As Well As Setting Direction
    private DcMotor setupDriveMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return aMotor;
    }



    public void showTelemetry(boolean show) {
        showTelemetry = show;
    }
    public void TelemetryOutput() {

        myOpMode.telemetry.addData("Red_Value", "%.2f", redValue);
        myOpMode.telemetry.addData("Green_Value", "%.2f", greenValue);
        myOpMode.telemetry.addData("Blue_Value", "%.2f", blueValue);
        myOpMode.telemetry.addData("Alpha_Value", "%.2f", alphaValue);
        myOpMode.telemetry.addData("Color", intakeColor);

    }

    public void Deposit_Wrist(boolean t) {

    }

    public void Setup_Vertical_Lift(int EXT, double pow) {
        VL_Extension = EXT;

    }


    public void Setup_Intake_Pose(double Rot) {
     //   I_Rotation = Rot;
     //   IL.setPosition(I_Rotation);
     //   IR.setPosition(I_Rotation);
    }

    public void Setup_Intake_Pose_RTP(boolean t) {
        if(t) {
           // IL.setPosition(0.45);
           // IR.setPosition(0.45);
        } else{
           // IL.setPosition(0.0);
           // IR.setPosition(0.0);
        }
    }

    public void Setup_Deposit_Claw(boolean t) {
        if(t){
            //DC.setPosition(0.3);
        } else {
            //DC.setPosition(0.0);
        }
    }



    public void Intake_Poop(boolean t) {
    }

    public void Setup_Deposit_Arm(double Rot){

    }


    public void verticalSlideUp(){
        Setup_Vertical_Lift(800, 1.0);
    }


    public void TransferSample(){
    }
}
