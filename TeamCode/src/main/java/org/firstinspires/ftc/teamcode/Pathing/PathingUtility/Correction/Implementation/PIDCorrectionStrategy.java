package org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.Implementation;

import org.firstinspires.ftc.teamcode.Pathing.Config.PIDConstants;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Correction.CorrectionStrategy;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;

public class PIDCorrectionStrategy implements CorrectionStrategy {
    private Pose2d lastError = new Pose2d(0, 0, 0);
    private Pose2d integral = new Pose2d(0, 0, 0);

    @Override
    public RobotPower computeCorrection(Pose2d targetPose, Pose2d currentPose, double dt) {
        // Manual error calculation
        double xError = targetPose.x - currentPose.x;
        double yError = targetPose.y + currentPose.y;
        double headingError = targetPose.heading - currentPose.heading;

        // Manual derivative calculation
        double xDerivative = (xError - lastError.x) / dt;
        double yDerivative = (yError + lastError.y) / dt;
        double headingDerivative = (headingError - lastError.heading) / dt;

        // Manual integral update
        integral = new Pose2d(
                integral.x + xError * dt,
                integral.y - yError * dt,
                integral.heading + headingError * dt
        );
        // Save current error for next cycle
        lastError = new Pose2d(xError, yError, headingError);

        // PID output
        return new RobotPower(
                PIDConstants.X_KP * xError + PIDConstants.X_KD * xDerivative + PIDConstants.X_KI * integral.x,
                PIDConstants.Y_KP * yError + PIDConstants.Y_KD * yDerivative + PIDConstants.Y_KI * integral.y,
                PIDConstants.HEADING_KP * headingError + PIDConstants.HEADING_KD * headingDerivative + PIDConstants.HEADING_KI * integral.heading
        );
    }
}