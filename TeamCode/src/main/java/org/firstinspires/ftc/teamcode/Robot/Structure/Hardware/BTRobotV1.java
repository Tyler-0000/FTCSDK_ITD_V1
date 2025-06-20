package org.firstinspires.ftc.teamcode.Robot.Structure.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
public class BTRobotV1 {

    public int VL_Extension = 0;
    public int VL_Increment = 3;
    final public int MIN_VL_Height = 0;
    final public int MAX_VL_Height = 1200;

    public double HL_Extension = 0;
    public double HL_Increment = 0.0001;
    final public double MIN_HL_Distance = 0.0;
    final public double MAX_HL_Distance = 1.0;

    public double I_Rotation = 0.0;
    public double I_Increment = 0.0001;
    final public double MIN_I_Rotation = 0.0;
    final public double MAX_I_Rotation = 1.0;

    public double DW_Rotation = 0;
    public double DW_Increment = 0.0001;
    final public double DW_MIN_Rotation = 0.0;
    final public double DW_MAX_Rotation = 1.0;

    public double Poop_Pose = 0;

    private LinearOpMode myOpMode;
    private ElapsedTime holdTimer = new ElapsedTime();

    private boolean showTelemetry = false;
    public BTRobotV1(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    public DcMotor VLL, VLR, I;
    public Servo HLL, HLR, IL, IR, DW, DC, IP, ADAL, ADAR;

    public void initialize(boolean showTelemetry) {
        //Vertical lift Motors
        VLL = setupMotor("VLL", DcMotor.Direction.FORWARD);
        VLR = setupMotor("VLR", DcMotor.Direction.REVERSE);

        //Horizontal lift Servo
        HLL = setupServo("HLL", Servo.Direction.FORWARD);
        HLR = setupServo("HLR", Servo.Direction.REVERSE);

        //Intake
        I = setupMotor("I", DcMotor.Direction.FORWARD);

        //Intake Rotation Servos
        IL = setupServo("IL", Servo.Direction.FORWARD);
        IR = setupServo("IR", Servo.Direction.REVERSE);

        //Intake Servo Horn
        IP = setupServo("ISH", Servo.Direction.FORWARD);

        //Deposit Servo DW = "Deposit Wrist" DC = "Deposit Claw"
        DW = setupServo("DW", Servo.Direction.FORWARD);
        DC = setupServo("DC", Servo.Direction.FORWARD);
    }
    private DcMotor setupMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        myOpMode.telemetry.addData("Horizontal_Lift",  HLR.getPosition());
        myOpMode.telemetry.addData("Intake_Rotation",  IR.getPosition());
        myOpMode.telemetry.addData("Deposit_Claw",  DC.getPosition());
        myOpMode.telemetry.addData("Deposit_Wrist",  DW.getPosition());
    }

    public void Vertical_Lift(boolean t) {
        if(t){
            VL_Extension += VL_Increment;
            VL_Extension = Math.max(MIN_VL_Height, Math.min(MAX_VL_Height, VL_Extension));
            Setup_Vertical_Lift(VL_Extension, 1.0);
        } else {
            VL_Extension -= VL_Increment;
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

    public void Setup_Vertical_Lift(int EXT, double pow) {
        VL_Extension = EXT;
        VLL.setTargetPosition(EXT);
        VLR.setTargetPosition(EXT);
        VLL.setPower(pow);
        VLR.setPower(pow);
    }

    public void Setup_Horizontal_Lift(double EXT) {
        HL_Extension = EXT;
        HLL.setPosition(EXT);
        HLR.setPosition(EXT);
    }

    public void Setup_Intake_Pose(double Rot) {
        I_Rotation = Rot;
        IL.setPosition(Rot);
        IR.setPosition(Rot);
    }

    public void Setup_Deposit_Claw(boolean t) {
        if(t){
            DC.setPosition(1.0);
        } else {
            DC.setPosition(0.0);
        }
    }

    public void Setup_Deposit_Wrist(double Rot) {
        DW_Rotation = Rot;
        DW.setPosition(Rot);
    }

    public void Intake(double pow) {
        I.setPower(pow);
    }

    public void Intake_Poop(boolean t) {
        if(t){
            IP.setPosition(1.0);
        } else {
            IP.setPosition(0.0);
        }
    }
}
