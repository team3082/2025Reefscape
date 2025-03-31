package frc.robot.utils.trajectories;

import java.util.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.utils.Vector2;

/**
 * Represents a path composed of a series of points, supporting operations like splitting,
 * finding the closest point, computing drive vectors, and calculating path length.
 */
public class Path {
    private final NavigableMap<Double, Point> points;
    private final double totalLength;

    /**
     * Constructs a Path object from a navigable map of t-values to points.
     *
     * @param points A map of t-values to points defining the path.
     * @throws IllegalArgumentException if the provided map is empty.
     */
    public Path(NavigableMap<Double, Point> points) {
        if (points.isEmpty()) {
            throw new IllegalArgumentException("Path cannot be empty");
        }
        this.points = new TreeMap<>(points);
        this.totalLength = calculateTotalLength();
    }

    /**
     * Calculates the total length of the path by summing distances between consecutive points.
     *
     * @return The total length of the path.
     */
    private double calculateTotalLength() {
        double totalLength = 0;
        List<Point> pointList = new ArrayList<>(points.values());
        for (int i = 0; i < pointList.size() - 1; i++) {
            Point p1 = pointList.get(i);
            Point p2 = pointList.get(i + 1);
            totalLength += p1.getPosition().sub(p2.getPosition()).mag();
        }
        return totalLength;
    }

    /**
     * Splits the path into multiple sub-paths based on the given t-values.
     *
     * @param splitTValues A list of t-values where the path should be split.
     * @return A list of navigable maps representing the sub-paths.
     * @throws IllegalArgumentException if any split t-value is out of bounds (less than min or greater than max).
     */
    public ArrayList<Path> splitPath(List<Double> splitTValues) {
        List<NavigableMap<Double, Point>> subPaths = new ArrayList<>();
        double minT = points.firstKey();
        double maxT = points.lastKey();
        double lastT = minT;

        for (double t : splitTValues) {
            // Validate if t is within the bounds
            if (Math.round(t*100) < Math.round(minT*100) ||  Math.round(lastT*100) > Math.round(maxT*100)) {
                throw new IllegalArgumentException("Split t-value out of bounds: " + t);
            }

            // Add the sub-path from lastT to t (inclusive)
            subPaths.add(new TreeMap<>(points.subMap(lastT, true, t, true)));
            lastT = t;
        }

        // Add the final sub-path from lastT to maxT
        subPaths.add(new TreeMap<>(points.tailMap(lastT, true)));

        ArrayList<Path> paths = new ArrayList<>();
        for (NavigableMap<Double, Point> subPathMap : subPaths) {
            paths.add(new Path(subPathMap));
        }

        return paths;
    }

    
    /**
     * Finds the closest t-value on the path to the given robot position.
     *
     * @param robotPosition The current position of the robot.
     * @return The t-value of the closest point.
     */
    public double findClosestT(Vector2 robotPosition) {
        return points.keySet().stream()
                .min(Comparator.comparingDouble(t -> points.get(t).getPosition().dist(robotPosition)))
                .orElseThrow(() -> new NoSuchElementException("No closest point found, THIS SHOULD NOT BE POSSIBLE, PLEASEEE LOOK AT THIS"));
    }

    /**
     * Computes the remaining length of the path from the given t-value.
     *
     * @param currentT The current t-value.
     * @return The remaining length of the path.
     */
    public double computeRemainingLength(double currentT) {
        if (!points.containsKey(currentT)) {
            throw new IllegalArgumentException("The provided t-value is not in the path: " + currentT);
        }

        NavigableMap<Double, Point> remainingPath = points.tailMap(currentT, true);

        if (remainingPath.size() <= 1) return 0;

        List<Point> remainingPoints = new ArrayList<>(remainingPath.values());

        double totalLength = 0;
        for (int i = 0; i < remainingPoints.size() - 1; i++) {
            Point p1 = remainingPoints.get(i);
            Point p2 = remainingPoints.get(i + 1);
            totalLength += p1.getPosition().sub(p2.getPosition()).mag();
        }

        return totalLength;
    }


    /**
     * Finds the nearest follow point on the path using a t-value lookahead.
     *
     * @param currentT The current t-value.
     * @param tLookAhead The t-value lookahead distance.
     * @return The nearest follow point on the path.
     */
    public Point getNearestFollowPoint(double currentT, double tLookAhead) {
        Double followT = points.ceilingKey(currentT + tLookAhead);
        if (followT == null) followT = points.lastKey();
        return points.get(followT);
    }

    /**
     * Gets the total length of the path.
     *
     * @return The total length of the path.
     */
    public double getTotalLength() { return totalLength; }

    /**
     * Gets the first point on the path.
     *
     * @return The first point of the path.
     */
    public Point getFirstPoint() { return points.get(points.firstKey()); }

    /**
     * Gets the last point on the path.
     *
     * @return The last point of the path.
     */
    public Point getLastPoint() { return points.get(points.lastKey()); }
    
    public Trajectory getTrajectory() {
        // Ensure the path has enough points
        if (points.size() < 2) {
            throw new IllegalArgumentException("Path must have at least two points to generate a trajectory.");
        }
    
        // Convert the first and last points to Pose2d (start & end)
        Point firstPoint = getFirstPoint();
        Point lastPoint = getLastPoint();
    
        Pose2d startPose = new Pose2d(
            new Translation2d(firstPoint.getPosition().x, firstPoint.getPosition().y),
            new Rotation2d(firstPoint.getRotation())
        );
    
        Pose2d endPose = new Pose2d(
            new Translation2d(lastPoint.getPosition().x, lastPoint.getPosition().y),
            new Rotation2d(lastPoint.getRotation())
        );
    
        // Convert intermediate points to waypoints
        List<Translation2d> waypoints = new ArrayList<>();
        int index = 0;
        for (Point point : points.values()) {
            if(point.getPosition().equals(firstPoint.getPosition()) || point.getPosition().equals(lastPoint.getPosition())) continue;
            if(index < 5){
                index ++;
                continue;
            }
            index = 0;
            waypoints.add(new Translation2d(point.getPosition().x, point.getPosition().y));
        }
    
        // Define trajectory configuration (adjust constraints as needed)
        TrajectoryConfig config = new TrajectoryConfig(2.0, 2.0);// Max velocity & acceleration
            
        // Generate and return the trajectory
        return TrajectoryGenerator.generateTrajectory(startPose, waypoints, endPose, config);
    }
    
}
