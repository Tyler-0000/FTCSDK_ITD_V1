package org.firstinspires.ftc.teamcode.Pathing.RobotUtility;

public class Vector2d {
    public final double x;
    public final double y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getMagnitude() {
        return Math.hypot(x, y);
    }

    public Vector2d normalize() {
        double mag = getMagnitude();
        return mag == 0 ? new Vector2d(0, 0) : new Vector2d(x / mag, y / mag);
    }

    public Vector2d scale(double factor) {
        return new Vector2d(x * factor, y * factor);
    }

    public Vector2d add(Vector2d other) {
        return new Vector2d(this.x + other.x, this.y + other.y);
    }

    public Vector2d subtract(Vector2d other) {
        return new Vector2d(this.x - other.x, this.y - other.y);
    }

    public double dot(Vector2d other) {
        return this.x * other.x + this.y * other.y;
    }

    public double cross(Vector2d other) {
        return this.x * other.y - this.y * other.x;
    }

    public Vector2d rotate(double angleRadians) {
        double cos = Math.cos(angleRadians);
        double sin = Math.sin(angleRadians);
        return new Vector2d(x * cos - y * sin, x * sin + y * cos);
    }

    public Vector2d negate() {
        return new Vector2d(-x, -y);
    }

    public boolean isZero() {
        return Math.abs(x) < 1e-6 && Math.abs(y) < 1e-6;
    }

    public double distanceTo(Vector2d other) {
        double dx = this.x - other.x;
        double dy = this.y - other.y;
        return Math.hypot(dx, dy);
    }

    public double angleTo(Vector2d other) {
        double dot = this.dot(other);
        double magProduct = this.getMagnitude() * other.getMagnitude();
        if (magProduct == 0) return 0;
        return Math.acos(clamp(dot / magProduct, -1.0, 1.0));
    }

    public Vector2d projectOnto(Vector2d other) {
        double scale = this.dot(other) / other.dot(other);
        return other.scale(scale);
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    @Override
    public String toString() {
        return String.format("Vector2d(x=%.3f, y=%.3f)", x, y);
    }
}