package org.firstinspires.ftc.teamcode.Pathing.PathingGeneration;

import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Utility for comparing expected vs. actual robot paths.
 * Assumes:
 *   - x = forward
 *   - y = strafe
 */
public class TrajectoryDebugger {

    private final List<Pose2d> expectedPath = new ArrayList<>();
    private final List<Pose2d> actualPath = new ArrayList<>();

    /** Samples the expected trajectory at evenly spaced intervals. */
    public void sampleExpected(Trajectory trajectory, int count) {
        expectedPath.clear();
        expectedPath.addAll(trajectory.getSamples(count));
    }

    /** Logs the actual robot pose at runtime. */
    public void logActual(Pose2d pose) {
        actualPath.add(pose);
    }

    /**
     * Compares expected and actual paths and logs error metrics.
     * Reports positional and heading error at each sample point.
     */
    public void comparePaths(Telemetry telemetry) {
        int count = Math.min(expectedPath.size(), actualPath.size());
        if (count == 0) {
            telemetry.addLine("No path data to compare.");
            telemetry.update();
            return;
        }

        double totalError = 0;

        for (int i = 0; i < count; i++) {
            Pose2d expected = expectedPath.get(i);
            Pose2d actual = actualPath.get(i);

            double dx = expected.x - actual.x;       // forward error
            double dy = expected.y - actual.y;       // strafe error
            double headingError = Math.abs(normalizeAngle(expected.heading - actual.heading));

            double distanceError = Math.hypot(dx, dy);
            totalError += distanceError;

            telemetry.addData("t=" + i,
                    "Pos Error: %.2f in | Heading Error: %.2f°",
                    distanceError, Math.toDegrees(headingError));
        }

        double avgError = totalError / count;
        telemetry.addData("Average Positional Error", "%.2f inches", avgError);
        telemetry.update();
    }

    /** Returns the sampled expected path. */
    public List<Pose2d> getExpectedPath() {
        return expectedPath;
    }

    /** Returns the logged actual path. */
    public List<Pose2d> getActualPath() {
        return actualPath;
    }

    /** Normalizes angle to [-π, π] range. */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}