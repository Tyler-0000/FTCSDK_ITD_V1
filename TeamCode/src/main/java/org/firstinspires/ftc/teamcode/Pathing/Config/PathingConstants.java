package org.firstinspires.ftc.teamcode.Pathing.Config;

public class PathingConstants {
    // Sampling resolution for trajectory evaluation (inches)
    public static final double SAMPLE_RESOLUTION = 1.0; // inches

    // Velocity thresholds in inches per second
    public static final double MIN_VELOCITY = 4.0;
    public static final double MAX_ROTATION_VELOCITY = Math.PI; // rad/s

    // Smoothing factor for curve interpolation
    public static final double CURVE_SMOOTHING_FACTOR = 0.8;

    // Minimum path length required to accept a trajectory
    public static final double MIN_PATH_LENGTH = 6.0; // inches

    // Default duration used for fallback timing or debugging overlays
    public static final double DEFAULT_PATH_DURATION = 2.5; // seconds

    // Threshold for considering robot "near end" of trajectory
    public static final double END_THRESHOLD_DISTANCE = 3.0; // inches

    // Velocity threshold for enabling integral correction
    public static final double LOW_VELOCITY_THRESHOLD = 3.0;

    // Integral increment when error persists near endpoint
    public static final double INTEGRAL_INCREMENT = 0.002;

    // Error threshold for triggering deviation recovery
    public static final double DEVIATION_THRESHOLD = 3.0;

    // Maximum allowable pose error before logging warning
    public static final double MAX_POSE_ERROR = 6.0;

    // Lookahead offset for heading correction
    public static final double HEADING_LOOKAHEAD_DISTANCE = 6.0; // inches

    // Clamp limits for PID output (optional, if not handled elsewhere)
    public static final double MAX_CORRECTION_VELOCITY = 40.0;
}