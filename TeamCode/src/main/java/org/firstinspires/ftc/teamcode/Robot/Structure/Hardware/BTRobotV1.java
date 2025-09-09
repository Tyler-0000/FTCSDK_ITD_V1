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
     public String intakeColor;


    private OpMode myOpMode;
    private ElapsedTime holdTimer = new ElapsedTime();

    private boolean showTelemetry = false;
    public BTRobotV1(OpMode opMode) {
        myOpMode = opMode;
    }

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


}
