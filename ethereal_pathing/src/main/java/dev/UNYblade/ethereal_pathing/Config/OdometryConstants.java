package dev.UNYblade.ethereal_pathing.Config;

public class OdometryConstants {
    public static final double TICKS_PER_REV = 2000; // from goBILDA spec
    public static final double WHEEL_DIAMETER = 1.26; // inches
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double GEAR_RATIO = 1.0;

    //Distance from left pod to right pod
    public static final double TRACK_WIDTH = 14.5; // tune this
    //Distance from strafe pod to center
    public static final double STRAFE_OFFSET = 7.25; // tune this
}
