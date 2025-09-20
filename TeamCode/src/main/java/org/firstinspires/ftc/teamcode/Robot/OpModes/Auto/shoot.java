package org.firstinspires.ftc.teamcode.Robot.OpModes.Auto;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "ty")
public class shoot extends LinearOpMode {
    private DcMotor fl, bl;

    @Override
    public void runOpMode(){
        fl  = hardwareMap.get(DcMotor.class, "leftFront");
        bl  = hardwareMap.get(DcMotor.class, "leftRear");
        waitForStart();
        while(opModeIsActive()){
            fl.setPower(1.0);
            bl.setPower(1.0);
        }
    }
}
