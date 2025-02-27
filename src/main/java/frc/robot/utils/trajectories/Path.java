package frc.robot.utils.trajectories;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Tuning;
import frc.robot.utils.Vector2;

/**
 * Represents a path composed of a series of points. This class provides functionality
 * to calculate the total length of the path, split it into sub-paths, find the closest
 * point on the path to a given position, and compute drive vectors for navigation.
 */
public class Path {
    private final List<Point> points;
    private final double totalLength;

    /**
     * Constructs a new Path object using the provided list of points.
     *
     * @param points The list of points that define the path.
     */
    public Path(List<Point> points) {
        this.points = new ArrayList<>(points);
        this.totalLength = calculateTotalLength();
    }

    private double calculateTotalLength() {
        double length = 0.0;
        for (int i = 1; i < points.size(); i++) {
            length += points.get(i - 1).getPosition().dist(points.get(i).getPosition());
        }
        return length;
    }

    /**
     * Splits the path into multiple sub-paths based on the provided split points.
     *
     * @param splitTValues A list of t-values where the path should be split.
     * @return A list of sub-paths created by splitting the original path.
     */
    public List<Path> splitPath(List<Double> splitTValues) {
        double minT = points.get(0).getT();
        double maxT = points.get(points.size() - 1).getT();

        for (double t : splitTValues) {
            if (t < minT || t > maxT) {
                throw new IllegalArgumentException("Split t-value out of bounds: " + t);
            }
        }

        double lastStopPoint = minT;
        List<Path> subPaths = new ArrayList<>();
        for (double t : splitTValues) {
            subPaths.add(new Path(extractPoints(lastStopPoint, t)));
            lastStopPoint = t;
        }
        subPaths.add(new Path(extractPoints(lastStopPoint, maxT)));
        return subPaths;
    }

    /**
 * Finds the closest point on the path to the given robot position.
 *
 * @param currentT The starting t-value.
 * @param robotPosition The robot's current position.
 * @return The t-value of the closest point on the path, which is never less than currentT.
 */
public double findClosestT(double currentT, Vector2 robotPosition) {
    double closestT = currentT;
    double minDistance = Double.MAX_VALUE;

    // Replace enhanced for-loop with standard for-loop
    for (int i = 0; i < points.size(); i++) {
        Point point = points.get(i);  // Get the Point from the list by index
        double t = point.getT();
        double distance = point.getPosition().dist(robotPosition);

        if (distance < minDistance && t >= currentT) {  // Ensure t is not less than currentT
            minDistance = distance;
            closestT = t;
        }
    }
    return closestT;
}

    /**
     * Computes the remaining length of the path from the given t-value.
     *
     * @param currentT The current t-value.
     * @return The remaining path length.
     */
    public double computeRemainingLength(double currentT) {
        double length = 0.0;
        Vector2 lastPoint = null;

        for (Point point : points) {
            if (point.getT() >= currentT) {
                Vector2 position = point.getPosition();
                if (lastPoint != null) {
                    length += position.sub(lastPoint).mag();
                }
                lastPoint = position;
            }
        }
        return length;
    }

    /**
     * Computes the drive vector to move towards a target point on the path.
     *
     * @param currentT The current t-value.
     * @param lookahead The number of points ahead to target.
     * @param currentPosition The robot's current position.
     * @return A normalized vector pointing to the target point.
     */
    public Vector2 computeDriveVector(double currentT, int lookahead, Vector2 currentPosition) {
        Point targetPoint = points.get(0);
        for (Point point : points) {
            if (point.getT() >= currentT) {
                targetPoint = point;
                break;
            }
        }
        return targetPoint.getPosition().sub(currentPosition).norm();
    }

    /**
     * Retrieves a subset of points between two t-values.
     *
     * @param startT The starting t-value.
     * @param endT The ending t-value.
     * @return A list of points between the given t-values.
     */
    private List<Point> extractPoints(double startT, double endT) {
        List<Point> subPoints = new ArrayList<>();
        for (Point point : points) {
            if (point.getT() >= startT && point.getT() <= endT) {
                subPoints.add(point);
            }
        }
        return subPoints;
    }

    public double getTotalLength() {
        return totalLength;
    }

    public Point getFirstPoint() {
        return points.get(0);
    }

    public Point getNearestFollowPoint(double currentT, int lookAhead) {
        Point closestPoint = points.get(0);
        int closeIndex = 0;
        for (int index = 0; index < points.size(); index ++) {
            if (Math.abs(points.get(index).getT() - currentT) < Math.abs(closestPoint.getT() - currentT)) {
                closestPoint = points.get(index);
                closeIndex=index;
            }
        }

        if(closeIndex + lookAhead > points.size()-1) return closestPoint;
        return points.get(closeIndex+lookAhead);
    }

    public Point getLastPoint() {
        return points.get(points.size() - 1);
    }
}
