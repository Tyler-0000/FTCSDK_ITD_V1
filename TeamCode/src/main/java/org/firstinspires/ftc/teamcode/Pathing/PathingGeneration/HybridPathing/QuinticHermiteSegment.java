// Package declaration for organizing the class within the HybridPathing module
package org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing;

// Importing constants for field dimensions
import org.firstinspires.ftc.teamcode.Pathing.Config.FieldConstants;
// Importing constants for pathing configuration (e.g., sample resolution)
import org.firstinspires.ftc.teamcode.Pathing.Config.PathingConstants;
// Importing the Waypoint class which holds position and derivative data
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Waypoint;
// Importing Pose2d class to represent position and heading
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

// Class representing a segment of a path using a quintic Hermite spline
public class QuinticHermiteSegment implements CurveSegment {
    // Start and end waypoints for the spline segment
    private final Waypoint start, end;

    // Constructor initializes the segment with two waypoints
    public QuinticHermiteSegment(Waypoint start, Waypoint end) {
        this.start = start;
        this.end = end;

        // Validate that the waypoints are within field boundaries
        validateWaypoint(start);
        validateWaypoint(end);
    }

    // Returns the interpolated pose (position and heading) at parameter t [0,1]
    @Override
    public Pose2d getPose(double t) {
        // Interpolate x-coordinate using quintic Hermite spline
        double x = quinticInterpolate(start.getX(), start.getDx(), start.getDdx(),
                end.getX(), end.getDx(), end.getDdx(), t);
        // Interpolate y-coordinate using quintic Hermite spline
        double y = quinticInterpolate(start.getY(), start.getDy(), start.getDdy(),
                end.getY(), end.getDy(), end.getDdy(), t);
        // Return the pose with interpolated position and computed heading
        return new Pose2d(x, y, getHeading(t));
    }

    // Computes the heading (orientation) at parameter t based on derivatives
    @Override
    public double getHeading(double t) {
        // Compute x-derivative at t
        double dx = quinticDerivative(start.getX(), start.getDx(), start.getDdx(),
                end.getX(), end.getDx(), end.getDdx(), t);
        // Compute y-derivative at t
        double dy = quinticDerivative(start.getY(), start.getDy(), start.getDdy(),
                end.getY(), end.getDy(), end.getDdy(), t);
        // Return heading as the angle of the tangent vector
        return Math.atan2(dy, dx);
    }

    // Approximates the arc length of the curve by sampling points
    @Override
    public double getLength() {
        double length = 0;
        // Start with pose at t = 0
        Pose2d prev = getPose(0);
        // Sample the curve at regular intervals and sum distances
        for (double t = PathingConstants.SAMPLE_RESOLUTION; t <= 1.0; t += PathingConstants.SAMPLE_RESOLUTION) {
            Pose2d curr = getPose(t);
            length += curr.distanceTo(prev); // Add distance between consecutive poses
            prev = curr;
        }
        return length;
    }

    // Returns the normalized tangent vector and heading at parameter t
    @Override
    public Pose2d getTangent(double t) {
        // Compute x and y derivatives
        double dx = quinticDerivative(start.getX(), start.getDx(), start.getDdx(),
                end.getX(), end.getDx(), end.getDdx(), t);
        double dy = quinticDerivative(start.getY(), start.getDy(), start.getDdy(),
                end.getY(), end.getDy(), end.getDdy(), t);

        // Compute heading from derivatives
        double heading = Math.atan2(dy, dx);
        // Compute magnitude of the tangent vector
        double magnitude = Math.sqrt(dx * dx + dy * dy);

        // Return normalized tangent vector with heading
        return magnitude == 0
                ? new Pose2d(0, 0, heading) // Zero vector if magnitude is zero
                : new Pose2d(dx / magnitude, dy / magnitude, heading); // Normalize
    }

    // Performs quintic Hermite interpolation for position at t
    private double quinticInterpolate(double p0, double v0, double a0,
                                      double p1, double v1, double a1, double t) {
        // Precompute powers of t
        double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
        // Return interpolated value using quintic Hermite formula
        return p0 + v0 * t + 0.5 * a0 * t2 +
                (-10 * p0 + 10 * p1 - 6 * v0 - 4 * v1 - 1.5 * a0 + 0.5 * a1) * t3 +
                (15 * p0 - 15 * p1 + 8 * v0 + 7 * v1 + 1.5 * a0 - a1) * t4 +
                (-6 * p0 + 6 * p1 - 3 * v0 - 3 * v1 - 0.5 * a0 + 0.5 * a1) * t5;
    }

    // Computes the derivative of the quintic Hermite spline at t
    private double quinticDerivative(double p0, double v0, double a0,
                                     double p1, double v1, double a1, double t) {
        // Precompute powers of t
        double t2 = t * t, t3 = t2 * t, t4 = t3 * t;
        // Return derivative using differentiated quintic Hermite formula
        return v0 + a0 * t +
                3 * (-10 * p0 + 10 * p1 - 6 * v0 - 4 * v1 - 1.5 * a0 + 0.5 * a1) * t2 +
                4 * (15 * p0 - 15 * p1 + 8 * v0 + 7 * v1 + 1.5 * a0 - a1) * t3 +
                5 * (-6 * p0 + 6 * p1 - 3 * v0 - 3 * v1 - 0.5 * a0 + 0.5 * a1) * t4;
    }

    // Validates that a waypoint is within the field boundaries
    private void validateWaypoint(Waypoint wp) {
        if (wp.getX() < 0 || wp.getX() > FieldConstants.FIELD_WIDTH ||
                wp.getY() < 0 || wp.getY() > FieldConstants.FIELD_HEIGHT) {
            System.out.println("Warning: Waypoint out of field bounds");
        }
    }
}