package org.firstinspires.ftc.teamcode.Pathing.Follower;

import org.firstinspires.ftc.teamcode.Pathing.Config.FieldConstants;
import org.firstinspires.ftc.teamcode.Pathing.Config.PIDConstants;
import org.firstinspires.ftc.teamcode.Pathing.Config.PathingConstants;
import org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.Trajectory;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.PIDController;
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

import java.util.ArrayList;
import java.util.List;

public class pathOperator {
    private final List<Trajectory> pathQueue = new ArrayList<>();
    private follower currentFollower = null;
    private int currentIndex = -1;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController headingController;

    public pathOperator() {
        this.xController = new PIDController(PIDConstants.X_KP, PIDConstants.X_KI, PIDConstants.X_KD);
        this.yController = new PIDController(PIDConstants.Y_KP, PIDConstants.Y_KI, PIDConstants.Y_KD);
        this.headingController = new PIDController(PIDConstants.HEADING_KP, PIDConstants.HEADING_KI, PIDConstants.HEADING_KD);
    }

    public void addPath(Trajectory trajectory) {
        Pose2d endPose = trajectory.getPoseAt(trajectory.getTotalTime());

        // Validate trajectory bounds
        if (endPose.getX() < 0 || endPose.getX() > FieldConstants.FIELD_WIDTH ||
                endPose.getY() < 0 || endPose.getY() > FieldConstants.FIELD_HEIGHT) {
            System.out.println("Rejected path: end pose out of bounds");
            return;
        }

        pathQueue.add(trajectory);
    }

    public void startPath(int index, double currentTime) {
        if (index >= 0 && index < pathQueue.size()) {
            Trajectory traj = pathQueue.get(index);

            // Filter out short-duration paths
            if (traj.getTotalTime() < PathingConstants.DEFAULT_PATH_DURATION) {
                System.out.println("Skipping short path");
                return;
            }

            currentIndex = index;
            currentFollower = new follower(traj);
            currentFollower.start(currentTime);
        }
    }

    public RobotPower update(Pose2d currentPose, double currentTime) {
        if (currentFollower == null) return RobotPower.zero();

        RobotPower rawPower = currentFollower.update(currentPose, currentTime);
        return rawPower.clampToLimits();
    }

    public boolean isCurrentPathFinished(double currentTime) {
        return currentFollower != null && currentFollower.isFinished(currentTime);
    }

    public int getCurrentIndex() {
        return currentIndex;
    }

    public int getTotalPaths() {
        return pathQueue.size();
    }

    public void clearPaths() {
        pathQueue.clear();
        currentFollower = null;
        currentIndex = -1;
    }
}