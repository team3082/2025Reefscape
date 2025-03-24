package frc.robot.utils.trajectories;

import static frc.robot.Tuning.CURVE_RESOLUTION;

import java.util.ArrayList;
import java.util.List;

import frc.robot.utils.Vector2;

public class CubicBezier implements Curve {
    private Vector2 a;
    private Vector2 b;
    private Vector2 c;
    private Vector2 d;

    private List<Vector2> points = new ArrayList<>();

    public CubicBezier(Vector2 a, Vector2 b, Vector2 c, Vector2 d) {
        System.out.println("Creating CubicBezier");

        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;

        getEvenlySpacedPoints();
    }

    public CubicBezier(List<Vector2> points) {
        System.out.println("Creating CubicBezier");
        this.points = points;
    }

    public void getEvenlySpacedPoints() {
        for (int i = 0; i < CURVE_RESOLUTION; i++) {
            points.add(getPoint((double) i / (double) CURVE_RESOLUTION));
        }
    }

    public Vector2 getPoint(double t) {
        double x = (Math.pow(1 - t,3) * a.x) +
        (3 * Math.pow(1 - t,2) * t * b.x) +
        (3 * (1 - t) * Math.pow(t,2) * c.x) +
        (Math.pow(t,3) * d.x);

        double y = (Math.pow(1 - t,3) * a.y) +
        (3 * Math.pow(1 - t,2) * t * b.y) +
        (3 * (1 - t) * Math.pow(t,2) * c.y) +
        (Math.pow(t,3) * d.y);

        Vector2 r = new Vector2(x, y);
        return r;
    }

    public List<Vector2> getPoints() {
        return points;
    }

    public CubicBezier reverse() {
        List<Vector2> reversed = new ArrayList<>();
        for (int i = points.size() - 1; i >= 0; i--) {
            reversed.add(points.get(i));
        }
        return new CubicBezier(reversed);
    }

    public CubicBezier flipHorizontal() {
        List<Vector2> flipped = new ArrayList<>();
        for (Vector2 point : points) {
            flipped.add(new Vector2(point.x, -point.y));
        }
        return new CubicBezier(flipped);
    }

    public CubicBezier rotate(double angle) {
        List<Vector2> rotated = new ArrayList<>();
        for (Vector2 point : points) {
            rotated.add(point.rotate(angle));
        }
        return new CubicBezier(rotated);
    }
}
