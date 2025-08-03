// Package declaration for organizing the class within the HybridPathing module
package org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing;

// Import field boundary constants
import org.firstinspires.ftc.teamcode.Pathing.Config.FieldConstants;
// Import pathing configuration constants (e.g., sample resolution)
import org.firstinspires.ftc.teamcode.Pathing.Config.PathingConstants;
// Import Waypoint class for position and derivative data
import org.firstinspires.ftc.teamcode.Pathing.PathingUtility.Waypoint;
// Import Pose2d class to represent position and heading
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

// Class representing a clamped cubic spline segment between two waypoints
public class ClampedCubicSegment implements CurveSegment {
    // Start and end waypoints for the segment
    private final Waypoint start, end;

    // Constructor initializes the segment with two waypoints
    public ClampedCubicSegment(Waypoint start, Waypoint end) {
        this.start = start;
        this.end = end;

        // Validate that the waypoints are within field boundaries
        validateWaypoint(start);
        validateWaypoint(end);
    }

    // Returns the interpolated pose (position and heading) at parameter t [0,1]
    @Override
    public Pose2d getPose(double t) {
        // Interpolate x-coordinate using clamped cubic spline
        double x = cubicInterpolate(start.getX(), start.getDx(), end.getX(), end.getDx(), t);
        // Interpolate y-coordinate using clamped cubic spline
        double y = cubicInterpolate(start.getY(), start.getDy(), end.getY(), end.getDy(), t);
        // Return pose with interpolated position and computed heading
        return new Pose2d(x, y, getHeading(t));
    }

    // Computes the heading (orientation) at parameter t based on derivatives
    @Override
    public double getHeading(double t) {
        // Compute x-derivative at t
        double dx = cubicDerivative(start.getX(), start.getDx(), end.getX(), end.getDx(), t);
        // Compute y-derivative at t
        double dy = cubicDerivative(start.getY(), start.getDy(), end.getY(), end.getDy(), t);
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
        double dx = cubicDerivative(start.getX(), start.getDx(), end.getX(), end.getDx(), t);
        double dy = cubicDerivative(start.getY(), start.getDy(), end.getY(), end.getDy(), t);

        // Compute heading from derivatives
        double heading = Math.atan2(dy, dx);
        // Compute magnitude of the tangent vector
        double magnitude = Math.sqrt(dx * dx + dy * dy);

        // Return normalized tangent vector with heading
        return magnitude == 0
                ? new Pose2d(0, 0, heading) // Zero vector if magnitude is zero
                : new Pose2d(dx / magnitude, dy / magnitude, heading); // Normalize
    }

    // Performs clamped cubic interpolation for position at t
    private double cubicInterpolate(double p0, double m0, double p1, double m1, double t) {
        // Precompute powers of t
        double t2 = t * t;
        double t3 = t2 * t;
        // Return interpolated value using cubic Hermite formula
        return (2 * t3 - 3 * t2 + 1) * p0
                + (t3 - 2 * t2 + t) * m0
                + (-2 * t3 + 3 * t2) * p1
                + (t3 - t2) * m1;
    }

    // Computes the derivative of the cubic Hermite spline at t
    private double cubicDerivative(double p0, double m0, double p1, double m1, double t) {
        // Precompute powers of t
        double t2 = t * t;
        // Return derivative using differentiated cubic Hermite formula
        return (6 * t2 - 6 * t) * p0
                + (3 * t2 - 4 * t + 1) * m0
                + (-6 * t2 + 6 * t) * p1
                + (3 * t2 - 2 * t) * m1;
    }

    // Validates that a waypoint is within the field boundaries
    private void validateWaypoint(Waypoint wp) {
        if (wp.getX() < 0 || wp.getX() > FieldConstants.FIELD_WIDTH ||
                wp.getY() < 0 || wp.getY() > FieldConstants.FIELD_HEIGHT) {
            System.out.println("Warning: Waypoint out of field bounds");
        }
    }
}