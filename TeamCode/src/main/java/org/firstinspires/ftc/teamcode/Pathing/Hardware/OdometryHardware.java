package org.firstinspires.ftc.teamcode.Pathing.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.CorrectionBlender;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.CorrectionScaler;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.CorrectionStrategy;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.DiagnosticLogger;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation.SimpleScalar;
import org.firstinspires.ftc.teamcode.Pathing.Optimization.Efficiency.EfficiencyModel;
import org.firstinspires.ftc.teamcode.Pathing.Optimization.Speed.SpeedOptimizer;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation.WeightedBlender;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OdometryHardware {
    public final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public final DcMotorEx encoderLeft, encoderRight, encoderYaw;

    private int lastLeftTicks = 0;
    private int lastRightTicks = 0;
    private int lastYawTicks = 0;

    public OdometryHardware(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftRear");
        backRight = hardwareMap.get(DcMotorEx.class, "rightRear");

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        setBrakeMode(true);

        for (DcMotorEx motor : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        encoderLeft = backLeft;
        encoderRight = backRight;
        encoderYaw = frontRight;

        stop();
        resetEncoders();
    }

    public void applyPower(
            RobotPower basePower,
            Pose2d targetPose,
            Pose2d currentPose,
            double dt,
            CorrectionStrategy strategy,
            CorrectionScaler scalar ,
            CorrectionBlender blender,
            DiagnosticLogger logger,
            EfficiencyModel efficiencyModel,
            SpeedOptimizer speedOptimizer,
            double batteryVoltage
    ) {
        RobotPower rawCorrection = strategy.computeCorrection(targetPose, currentPose, dt);
        RobotPower scaledCorrection = scalar.scale(rawCorrection, currentPose, dt);
        RobotPower blendedPower = blender.blend(basePower, scaledCorrection).clampToLimits();
        RobotPower efficientPower = efficiencyModel.optimize(blendedPower, currentPose, batteryVoltage);
        RobotPower optimizedSpeedPower = speedOptimizer.optimizeSpeed(efficientPower, currentPose, batteryVoltage);

        logger.log(currentPose, targetPose, rawCorrection, scaledCorrection, optimizedSpeedPower);

        double f = optimizedSpeedPower.forward;
        double s = optimizedSpeedPower.strafe;
        double t = optimizedSpeedPower.turn;

        frontLeft.setPower(f + s + t);
        frontRight.setPower(f - s - t);
        backLeft.setPower(f - s + t);
        backRight.setPower(f + s - t);
    }

    public void stop() {
        for (DcMotorEx motor : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setPower(0);
        }
    }

    public void resetEncoders() {
        for (DcMotorEx motor : new DcMotorEx[]{frontRight, backLeft, backRight}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public int getLeftTicks() {
        return encoderLeft.getCurrentPosition();
    }

    public int getRightTicks() {
        return encoderRight.getCurrentPosition();
    }

    public int getStrafeTicks() {
        return encoderYaw.getCurrentPosition();
    }

    public void logMotorPower(Telemetry telemetry) {
        telemetry.addData("FL Power", frontLeft.getPower());
        telemetry.addData("FR Power", frontRight.getPower());
        telemetry.addData("BL Power", backLeft.getPower());
        telemetry.addData("BR Power", backRight.getPower());
    }

    public String getEncoderSnapshot() {
        return String.format("L:%d R:%d S:%d", getLeftTicks(), getRightTicks(), getStrafeTicks());
    }

    public void setBrakeMode(boolean brake) {
        DcMotor.ZeroPowerBehavior behavior = brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        for (DcMotorEx motor : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(behavior);
        }
    }

    public void updateLastEncoderSnapshot() {
        lastLeftTicks = getLeftTicks();
        lastRightTicks = getRightTicks();
        lastYawTicks = getStrafeTicks();
    }

    public boolean isStalled(double minTicksPerCycle, double minPower) {
        int leftDelta = Math.abs(getLeftTicks() - lastLeftTicks);
        int rightDelta = Math.abs(getRightTicks() - lastRightTicks);
        int yawDelta = Math.abs(getStrafeTicks() - lastYawTicks);

        double avgDelta = (leftDelta + rightDelta + yawDelta) / 3.0;
        boolean lowMovement = avgDelta < minTicksPerCycle;

        boolean highPower = Math.abs(frontLeft.getPower()) > minPower ||
                Math.abs(frontRight.getPower()) > minPower ||
                Math.abs(backLeft.getPower()) > minPower ||
                Math.abs(backRight.getPower()) > minPower;

        return lowMovement && highPower;
    }

    public void attemptStallRecovery() {
        Pose2d currentPose = new Pose2d(0, 0, 0);
        Pose2d targetPose = new Pose2d(2, 0, 0);

        applyPower(
                new RobotPower(0.3, 0, 0),
                targetPose,
                currentPose,
                0.1,
                (t, c, d) -> new RobotPower(0.1, 0.05, 0.02), // mock correction
                new SimpleScalar(0.8, 0.8, 1.0),
                new WeightedBlender(0.7, 0.3),
                (c, t, r, s, f) -> {},
                (p, pose, v) -> p,
                (p, pose, v) -> p,
                12.0
        );

        try {
            Thread.sleep(150);
        } catch (InterruptedException ignored) {}

        stop();
    }
}