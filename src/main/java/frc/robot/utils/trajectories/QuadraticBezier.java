package frc.robot.utils.trajectories;

import static frc.robot.Tuning.CURVE_RESOLUTION;

import java.util.ArrayList;
import java.util.List;

import frc.robot.utils.Vector2;

public class QuadraticBezier implements Curve {
    private Vector2 a;
    private Vector2 b;
    private Vector2 c;

    private List<Vector2> points = new ArrayList<>();

    public QuadraticBezier(Vector2 a, Vector2 b, Vector2 c) {
        System.out.println("Creating QuadraticBezier");
        this.a = a;
        this.b = b;
        this.c = c;

        getEvenlySpacedPoints();
    }

    public QuadraticBezier(List<Vector2> points) {
        System.out.println("Creating QuadraticBezier");
        this.points = points;
    }

    public void getEvenlySpacedPoints() {
        for (int i = 0; i < CURVE_RESOLUTION; i++) {
            points.add(getPoint((double) i / (double) CURVE_RESOLUTION));
        }
    }

    public Vector2 getPoint(double t) {
        double x = (b.x + (Math.pow((1-t), 2) * (a.x - b.x)) + (Math.pow(t, 2) * (c.x - b.x)));
        double y = (b.y + (Math.pow((1-t), 2) * (a.y - b.y)) + (Math.pow(t, 2) * (c.y - b.y)));

        Vector2 r = new Vector2(x, y);
        return r;
    }

    public List<Vector2> getPoints() {
        return points;
    }

    public QuadraticBezier reverse() {
        List<Vector2> reversed = new ArrayList<>();
        for (int i = points.size() - 1; i >= 0; i--) {
            reversed.add(points.get(i));
        }
        return new QuadraticBezier(reversed);
    }

    public QuadraticBezier flipHorizontal() {
        List<Vector2> flipped = new ArrayList<>();
        for (Vector2 point : points) {
            flipped.add(new Vector2(point.x, -point.y));
        }
        return new QuadraticBezier(flipped);
    }

    public QuadraticBezier rotate(double angle) {
        List<Vector2> rotated = new ArrayList<>();
        for (Vector2 point : points) {
            rotated.add(point.rotate(angle));
        }
        return new QuadraticBezier(rotated);
    }
}
