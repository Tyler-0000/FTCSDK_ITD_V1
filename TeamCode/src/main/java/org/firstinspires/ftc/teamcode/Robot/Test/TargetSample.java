package org.firstinspires.ftc.teamcode.Robot.Test;

import org.opencv.core.Point;

import org.firstinspires.ftc.teamcode.Robot.Test.Vector2D;

public class TargetSample {

    Vector2D targetPoint;
    double angle;

    public TargetSample(Vector2D targetPoint, double angle){
        this.targetPoint = targetPoint;
        this.angle = angle;
    }

    public Vector2D getTargetPoint() {
        return targetPoint;
    }

    public double getAngle() {
        return angle;
    }
}
