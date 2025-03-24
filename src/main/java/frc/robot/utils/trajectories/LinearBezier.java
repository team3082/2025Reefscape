package frc.robot.utils.trajectories;

import static frc.robot.Tuning.CURVE_RESOLUTION;

import java.util.ArrayList;
import java.util.List;

import frc.robot.utils.Vector2;

public class LinearBezier implements Curve {
    private Vector2 a;
    private Vector2 b;

    private List<Vector2> points = new ArrayList<>();

    public LinearBezier(Vector2 a, Vector2 b) {
        System.out.println("Creating LinearBezier");
        this.a = a;
        this.b = b;

        getEvenlySpacedPoints();
    }

    public LinearBezier(List<Vector2> points) {
        System.out.println("Creating LinearBezier");
        this.points = points;
    }

    public void getEvenlySpacedPoints() {
        for (int i = 0; i < CURVE_RESOLUTION; i++) {
            points.add(getPoint((double) i / (double) CURVE_RESOLUTION));
        }
    }

    public Vector2 getPoint(double t) {
        double x = a.x + (t * (b.x - a.x));
        double y = a.y + (t * (b.y - a.y));

        Vector2 r = new Vector2(x, y);
        return r;
    }

    public List<Vector2> getPoints() {
        return points;
    }

    public LinearBezier reverse() {
        List<Vector2> reversed = new ArrayList<>();
        for (int i = points.size() - 1; i >= 0; i--) {
            reversed.add(points.get(i));
        }
        return new LinearBezier(reversed);
    }

    public LinearBezier flipHorizontal() {
        List<Vector2> flipped = new ArrayList<>();
        for (Vector2 point : points) {
            flipped.add(new Vector2(point.x, -point.y));
        }
        return new LinearBezier(flipped);
    }

    public LinearBezier rotate(double angle) {
        List<Vector2> rotated = new ArrayList<>();
        for (Vector2 point : points) {
            rotated.add(point.rotate(angle));
        }
        return new LinearBezier(rotated);
    }
}
