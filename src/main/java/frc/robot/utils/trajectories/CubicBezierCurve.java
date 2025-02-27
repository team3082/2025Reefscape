package frc.robot.utils.trajectories;

import java.util.ArrayList;
import java.util.List;
import static frc.robot.Tuning.CURVE_RESOLUTION;

import frc.robot.utils.Vector2;

/**
   * Represents a cubic Bezier curve defined by four control points.
   * This curve is computed based on the four control points using 
   * linear interpolation, providing methods to get points along 
   * the curve, the curve length, and other properties.
   */
public class CubicBezierCurve {
    private Vector2 controlPointOne;
    private Vector2 controlPointTwo; 
    private Vector2 controlPointThree; 
    private Vector2 controlPointFour;

    private ArrayList<Vector2> points = new ArrayList<>();

    /**
     * Constructs a CubicBezierCurve with the given control points.
     *
     * @param controlPointOne First control point.
     * @param controlPointTwo Second control point
     * @param controlPointThree Third control point
     * @param controlPointFour Fourth control point.
     */
    public CubicBezierCurve(Vector2 controlPointOne, Vector2 controlPointTwo, Vector2 controlPointThree, Vector2 controlPointFour) {
        this.controlPointOne = controlPointOne;
        this.controlPointTwo = controlPointTwo;
        this.controlPointThree = controlPointThree;
        this.controlPointFour = controlPointFour;

        for (int i = 0; i < CURVE_RESOLUTION; i++) {
            points.add(getPointAtT((double) i / (double) CURVE_RESOLUTION));
        }
    }

    public ArrayList<Vector2> getPoints(){
        return points;
    }

    /**
     * Evaluates the Bezier curve at a given parameter t.
     *
     * @param t A parameter value between 0 and 1.
     * @return The point on the curve corresponding to the given t value.
     * @throws IllegalArgumentException if t is outside the range [0, 1].
     */
    public Vector2 getPointAtT(double t) {
        if (t < 0 || t > 1) {
            throw new IllegalArgumentException("t must be between 0 and 1");
        }

        double x = (Math.pow(1 - t,3) * controlPointOne.x) +
        (3 * Math.pow(1 - t,2) * t * controlPointTwo.x) +
        (3 * (1 - t) * Math.pow(t,2) * controlPointThree.x) +
        (Math.pow(t,3) * controlPointFour.x);

        double y = (Math.pow(1 - t,3) * controlPointOne.y) +
        (3 * Math.pow(1 - t,2) * t * controlPointTwo.y) +
        (3 * (1 - t) * Math.pow(t,2) * controlPointThree.y) +
        (Math.pow(t,3) * controlPointFour.y);

        Vector2 curvePoint = new Vector2(x, y);
        return curvePoint;
    }

}