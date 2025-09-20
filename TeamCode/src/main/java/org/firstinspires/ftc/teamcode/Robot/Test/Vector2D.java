package org.firstinspires.ftc.teamcode.Robot.Test;

@SuppressWarnings("unused")
public class Vector2D {
    private double x, y;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2D() {
        this(0, 0);
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getNorm() {
        return Math.sqrt(x * x + y * y);
    }

    public double getMagnitude() {
        return Math.hypot(x, y);
    }

    public Vector2D set(double x, double y) {
        this.x = x;
        this.y = y;
        return this;
    }

    public Vector2D add(double x, double y) {
        return new Vector2D(this.x + x, this.y + y);
    }

    public Vector2D subtract(double x, double y) {
        return new Vector2D(this.x - x, this.y - y);
    }

    public Vector2D add(Vector2D other) {
        return new Vector2D(this.x + other.x, this.y + other.y);
    }

    public Vector2D subtract(Vector2D other) {
        return new Vector2D(this.x - other.x, this.y - other.y);
    }

    public Vector2D normalized() {
        double mag = getMagnitude();
        if (mag == 0) return new Vector2D(0, 0);
        return new Vector2D(x / mag, y / mag);
    }

    public double dot(Vector2D other) {
        return this.x * other.x + this.y * other.y;
    }

    public double angleTo(Vector2D other) {
        double dot = this.dot(other);
        double magProduct = this.getMagnitude() * other.getMagnitude();
        if (magProduct == 0) return 0;
        return Math.acos(clamp(dot / magProduct, -1.0, 1.0));
    }

    public double distanceTo(Vector2D other) {
        double dx = this.x - other.x;
        double dy = this.y - other.y;
        return Math.hypot(dx, dy);
    }

    public Vector2D scale(double scalar) {
        return new Vector2D(this.x * scalar, this.y * scalar);
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    @Override
    public String toString() {
        return String.format("(%.3f, %.3f)", x, y);
    }
}