// Package declaration for organizing the class within the HybridPathing module
package org.firstinspires.ftc.teamcode.Pathing.PathingGeneration.HybridPathing;

// Import field boundary constants
import org.firstinspires.ftc.teamcode.Pathing.Config.FieldConstants;
// Import pathing configuration constants (e.g., sample resolution)
import org.firstinspires.ftc.teamcode.Pathing.Config.PathingConstants;
// Import Pose2d class to represent position and heading
import org.firstinspires.ftc.teamcode.Pathing.RobotUtility.Pose2d;

// Class representing a cubic Bézier curve segment
public class BezierSegment implements CurveSegment {
    // Control points for the Bézier curve
    private final Pose2d p0, p1, p2, p3;

    // Constructor initializes the segment with four control points
    public BezierSegment(Pose2d p0, Pose2d p1, Pose2d p2, Pose2d p3) {
        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;

        // Validate that each control point is within field boundaries
        validatePose(p0);
        validatePose(p1);
        validatePose(p2);
        validatePose(p3);
    }

    // Returns the interpolated pose (position and heading) at parameter t [0,1]
    @Override
    public Pose2d getPose(double t) {
        // Interpolate x-coordinate using cubic Bézier formula
        double x = bezier(p0.getX(), p1.getX(), p2.getX(), p3.getX(), t);
        // Interpolate y-coordinate using cubic Bézier formula
        double y = bezier(p0.getY(), p1.getY(), p2.getY(), p3.getY(), t);
        // Return pose with interpolated position and computed heading
        return new Pose2d(x, y, getHeading(t));
    }

    // Computes the heading (orientation) at parameter t based on derivatives
    @Override
    public double getHeading(double t) {
        // Compute x-derivative at t
        double dx = bezierDerivative(p0.getX(), p1.getX(), p2.getX(), p3.getX(), t);
        // Compute y-derivative at t
        double dy = bezierDerivative(p0.getY(), p1.getY(), p2.getY(), p3.getY(), t);
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
        double dx = bezierDerivative(p0.getX(), p1.getX(), p2.getX(), p3.getX(), t);
        double dy = bezierDerivative(p0.getY(), p1.getY(), p2.getY(), p3.getY(), t);

        // Compute heading from derivatives
        double heading = Math.atan2(dy, dx);
        // Compute magnitude of the tangent vector
        double magnitude = Math.sqrt(dx * dx + dy * dy);

        // Return normalized tangent vector with heading
        return magnitude == 0
                ? new Pose2d(0, 0, heading) // Zero vector if magnitude is zero
                : new Pose2d(dx / magnitude, dy / magnitude, heading); // Normalize
    }

    // Performs cubic Bézier interpolation for position at t
    private double bezier(double a, double b, double c, double d, double t) {
        double u = 1 - t; // Complement of t
        // Return interpolated value using cubic Bézier formula
        return u * u * u * a + 3 * u * u * t * b + 3 * u * t * t * c + t * t * t * d;
    }

    // Computes the derivative of the cubic Bézier curve at t
    private double bezierDerivative(double a, double b, double c, double d, double t) {
        double u = 1 - t; // Complement of t
        // Return derivative using differentiated cubic Bézier formula
        return 3 * u * u * (b - a) + 6 * u * t * (c - b) + 3 * t * t * (d - c);
    }

    // Validates that a control point is within the field boundaries
    private void validatePose(Pose2d pose) {
        if (pose.getX() < 0 || pose.getX() > FieldConstants.FIELD_WIDTH ||
                pose.getY() < 0 || pose.getY() > FieldConstants.FIELD_HEIGHT) {
            System.out.println("Warning: Bézier control point out of field bounds");
        }
    }
}