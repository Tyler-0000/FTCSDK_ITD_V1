package org.firstinspires.ftc.teamcode.Robot.Test;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Limelight_Test", group = "Test")
public class Limelight_Test extends OpMode {

    private Limelight3A limelight;

    ArrayList<TargetSample> targetPoints = new ArrayList<>();

    public void setGettingResults(boolean gettingResults) {
        isGettingResults = gettingResults;
    }

    public boolean isGettingResults = true;
    double offset = 0;

    public ElapsedTime MovementTimer = new ElapsedTime();

    public double getCurrentTime() {
        return currentTime;
    }

    double currentTime = 0;

    public double OdometryTime = 0;
    public double limelight_elapsed_ms = 0;

    public void setDetectionColor(boolean detectingRed) {
        this.detectingRed = detectingRed;
        redDetection = detectingRed ? 1 : 0;
    }

    public boolean getAllianceSamples(){
        return detectingRed;
    }

    double odoTime = 0;

    public void setOdoTime(){
        odoTime = MovementTimer.milliseconds() - 40;
    }

    public enum color {
        red,
        yellow,
        blue
    }

    public void setAuto(boolean auto) {
        this.auto = auto;
    }

    boolean auto = false;

    public color getTargetColor() {
        return targetColor;
    }

    public void setTargetColor(color targetColor) {
        this.targetColor = targetColor;
        switch (targetColor){
            case red:
                limelight.pipelineSwitch(2);
                break;
            case yellow:
                if (auto) {
                    limelight.pipelineSwitch(1);
                } else {
                    limelight.pipelineSwitch(0);
                }
                break;
            case blue:
                limelight.pipelineSwitch(3);
                break;
            default:
        }
    }

    private color targetColor = color.yellow;

    public boolean detectingRed = true;

    public boolean isAllianceColor() {
        return allianceColor;
    }

    public void collectColoredSamples(boolean collecting) {
        this.allianceColor = collecting;
        allianceColorInt = allianceColor ? 1 : 0;
    }

    public boolean allianceColor = true;

    int redDetection = 1;
    int allianceColorInt = 0;

    public void setReturningData(boolean returningData) {
        this.returningData = returningData;

        if (returningData){
            limelight.start();
        }else {
            limelight.pause();
        }
    }

    public boolean returningData = false;

    public LLResult result;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);

        limelight.pipelineSwitch(0);

        onStart();
    }

    public void shutDown(){
        limelight.close();
    }

    public void onStart(){
        limelight.start();
    }

    @Override
    public void loop() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        setReturningData(true);

            if (returningData){

            result = limelight.getLatestResult();

            if (result != null && isGettingResults){
                targetPoints.clear();

                // Access general information
                Pose3D botpose = result.getBotpose();
                /*
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();

                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
                */
                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("tync", result.getTyNC());

                    telemetry.addData("Botpose", botpose.toString());
                }
            }
        }
    }

    public void updatePythonInputs(double X, double Y, double Heading, double SlideCM, double getXVelocity, double getYVelocity ){
        double[] inputs = {getXVelocity, redDetection, X, Y, Heading, SlideCM, allianceColorInt, getYVelocity};
        limelight.updatePythonInputs(inputs);
    }

    public void updateTimeStamp(){
        double[] inputs = {1, 2, 3, 4, 5, 6, 7, 8};
        limelight.updatePythonInputs(inputs);
        MovementTimer.reset();

        currentTime = 0;

        System.out.println("Sent update to Limelight");
    }

    public double getRobotTime(){
        return MovementTimer.milliseconds() - offset;
    }

    public void switchPipeline(int pipelineIndex){
        limelight.pipelineSwitch(pipelineIndex);
    }

    public TargetSample getTargetPoint(){
        if (!targetPoints.isEmpty()){
            return targetPoints.get(0);
        }else {
            return null;
        }

    }

    public TargetSample returnPointToCollect(){

        if (!targetPoints.isEmpty()){
            TargetSample targetSample = targetPoints.get(0);
            targetPoints.remove(0);
            return targetSample;
        }else {
            return null;
        }

    }
}
