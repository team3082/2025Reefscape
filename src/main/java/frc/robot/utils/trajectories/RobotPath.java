package frc.robot.utils.trajectories;

import static frc.robot.Tuning.CURVE_RESOLUTION;

import java.util.ArrayList;
import java.util.List;

import frc.robot.swerve.SwervePosition;
import frc.robot.utils.Vector2;

public class RobotPath {
    public List<Vector2> points;
    public double targetRot;

    public double currentPosT = 0;

    public RobotPath(List<Vector2> points, double targetRot) {
        this.points = points;
        this.targetRot = targetRot;
    }

    // update the closest point on the path from the robot's current position
    public void updatePosition(Vector2 robotPos) {
        double closestDist = -1.0;
        double closestT = 0;
        for (int i = (int) (currentPosT * (double) CURVE_RESOLUTION); i < points.size(); i++) {
            Vector2 point = points.get(i);
            if (point.sub(robotPos).mag() < closestDist) {
                closestDist = point.sub(robotPos).mag();
                closestT = (double) i / (double) CURVE_RESOLUTION;
            } else if (closestDist < 0) {
                closestDist = point.sub(robotPos).mag();
                closestT = (double) i / (double) CURVE_RESOLUTION;
            }
        }

        currentPosT = closestT;

        // System.out.println("currentPosT: " + currentPosT);
    }

    // return already calculated closest point
    public double getClosestT() {
        return currentPosT;
    }

    // get total length of path
    public double getPathLength() {
        Vector2 lastPoint = points.get(0);
        double length = 0;
        for (Vector2 point : points) {
            length += point.sub(lastPoint).mag();
            lastPoint = point;
        }

        // System.out.println("path length: " + length);
        return length;
    }

    // get lentgh of path from current position to end of path
    public double getRemainingPathLength() {
        double length = 0;
        Vector2 lastPoint = null;
        for (int i = (int) (currentPosT * (double) CURVE_RESOLUTION); i < points.size(); i++) {
            Vector2 point = points.get(i);
            if (lastPoint != null) {
                length += point.sub(lastPoint).mag();
            }
            lastPoint = point;
        }
        
        length += points.get((int) (currentPosT * (double) CURVE_RESOLUTION)).sub(SwervePosition.getPosition()).mag();
        
        return length;
    }

    // pure pursuit algorithm
    public Vector2 getDriveVector() {
        // get a vector of magnitude 1 that points in the direction of the next closest point from the robot's current position
        Vector2 targetPoint;
        try {
            targetPoint = points.get((int) (getClosestT() * CURVE_RESOLUTION) + 10);
        } catch (IndexOutOfBoundsException e) {
            targetPoint = points.get(points.size() - 1);
        }
        Vector2 driveVector = targetPoint.sub(SwervePosition.getPosition()).norm();
        return driveVector;
    }

    public double getTargetRot() {
        return targetRot;
    }

    public void addCurvePoints(List<Vector2> curvePoints) {
        points.addAll(curvePoints);
    }

    public List<Vector2> getCurvePoints(double startT, double endT) {
        int startTIndex = (int) (startT * (double) CURVE_RESOLUTION);
        int endTIndex = (int) (endT * (double) CURVE_RESOLUTION);

        return points.subList(startTIndex, endTIndex);
    }


    public List<RobotPath> separatePaths(List<Double> stopPointsList) {
        List<RobotPath> paths = new ArrayList<>();

        // gets all paths between each stop point
        double lastStopPoint = 0;
        for (double stopPoint : stopPointsList) {
            List<Vector2> curvePoints = getCurvePoints(lastStopPoint, stopPoint);
            paths.add(new RobotPath(curvePoints, targetRot));
            lastStopPoint = stopPoint;
        }

        // gets the very last path after the final specified stop point
        if (lastStopPoint < ((double) points.size() / (double) CURVE_RESOLUTION)) {
            List<Vector2> curvePoints = getCurvePoints(lastStopPoint, ((double) points.size() / (double) CURVE_RESOLUTION));
            paths.add(new RobotPath(curvePoints, targetRot));
        }

        return paths;
    }

    public Vector2 getStartPos() {
        return points.get(0);
    }

    public Vector2 getLastPos() {
        return points.get(points.size() - 1);
    }
}
