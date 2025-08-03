package dev.UNYblade.ethereal_pathing.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OdometryHardware {
    private final DcMotorEx leftEncoder;
    private final DcMotorEx rightEncoder;
    private final DcMotorEx strafeEncoder;

    public OdometryHardware(HardwareMap hardwareMap) {
        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightRear");
        strafeEncoder = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        strafeEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public double getLeftTicks() {
        return leftEncoder.getCurrentPosition();
    }

    public double getRightTicks() {
        return rightEncoder.getCurrentPosition();
    }

    public double getStrafeTicks() {
        return strafeEncoder.getCurrentPosition();
    }
}