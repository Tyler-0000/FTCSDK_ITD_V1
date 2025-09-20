package org.firstinspires.ftc.teamcode.Pathing.Config;

public class PIDConstants {
    // Primary path correction PID
    public static final double X_KP = 0.5;
    public static final double X_KI = 0.0;
    public static final double X_KD = 0.0;

    public static final double Y_KP = 0.5;
    public static final double Y_KI = 0.0;
    public static final double Y_KD = 0.0;

    public static final double HEADING_KP = 1.0;
    public static final double HEADING_KI = 0.0;
    public static final double HEADING_KD = 0.0;

    // Final endpoint correction PID (used near path end)
    public static final double X_KP_END = 1.5;
    public static final double X_KD_END = 0.2;

    public static final double Y_KP_END = 1.5;
    public static final double Y_KD_END = 0.2;
}