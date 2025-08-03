package dev.UNYblade.ethereal_pathing.Config;

public class PathingConstants {
    //Tune this
    public static final double SAMPLE_RESOLUTION = 0.02; // seconds

    // Velocity thresholds in inches per second
    //Tune this
    public static final double MIN_VELOCITY = 4.0; // ≈ 0.33 ft/sec
    //Tune this
    public static final double MAX_ROTATION_VELOCITY = Math.PI; // rad/s
    //Tune this
    public static final double CURVE_SMOOTHING_FACTOR = 0.8;
    //Tune this
    public static final double DEFAULT_PATH_DURATION = 5.0; // seconds
}