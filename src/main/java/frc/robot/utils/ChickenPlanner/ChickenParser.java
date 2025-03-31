package frc.robot.utils.ChickenPlanner;

import java.io.File;
import java.io.IOException;
import java.util.*;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.Tuning;
import frc.robot.utils.Vector2;
import frc.robot.utils.trajectories.Path;
import frc.robot.utils.trajectories.Point;
import frc.robot.utils.trajectories.CubicBezierCurve;
/**
 * Parses and processes trajectory data for a robot using control points and action points.
 */
public class ChickenParser {
    private static Map<String, ChickenTrajectory> trajectories = new HashMap<>();

    public static ChickenTrajectory getTrajectory(String name){
        if(!trajectories.containsKey(name)){
            throw new IllegalArgumentException("Trajectory "+name+" does not exist!");
        }
        return trajectories.get(name);
    }

    public static void init() {
        File directory = new File(Filesystem.getDeployDirectory(), "ChickenPlanner");
        File[] files = directory.listFiles((dir, name) -> name.toLowerCase().endsWith(".json"));
        if (files == null) return;

        new Thread(() -> {
            for (File file : files) {
                try {
                    trajectories.put(file.getName(), loadChickenPlannerTrajectory(file));
                    System.out.println(file.getName() + " processed");
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }, "Chicken Parser").start();
    }

    private static ChickenTrajectory loadChickenPlannerTrajectory(File file) throws IOException {
        System.out.println(file.getName()+" is being processed");
        ObjectMapper mapper = new ObjectMapper();
        JsonNode root = mapper.readTree(file);
        
        List<Point> points = processControlPoints(root.get("controlPoints"));
        List<ActionPoint> actionPoints = processActionPoints(root.get("actionPoints"));

        boolean stopAtEnd = false;
        for(int index = 0; index<actionPoints.size(); index++){
            if(actionPoints.get(index).t() > points.get(points.size()-5).getT()){
                actionPoints.remove(index);
                index--;
                stopAtEnd = true;
            }
        }

        assignRotations(points, actionPoints);

        Path wholePath = new Path(new TreeMap<>(mapPoints(points)));
        List<Double> splitPoints = actionPoints.stream().map(ActionPoint::t).toList();

        return new ChickenTrajectory(wholePath.splitPath(splitPoints), actionPoints, stopAtEnd);
    }

    private static List<Point> processControlPoints(JsonNode controlPoints) {
        List<CubicBezierCurve> bezierCurves = new ArrayList<>();
        for (int i = 0; i < (controlPoints.size() - 1) / 3; i++) {
            bezierCurves.add(new CubicBezierCurve(
                parseControlPoint(controlPoints.get(i * 3)),
                parseControlPoint(controlPoints.get(i * 3 + 1)),
                parseControlPoint(controlPoints.get(i * 3 + 2)),
                parseControlPoint(controlPoints.get(i * 3 + 3))
            ));
        }

        List<Point> points = new ArrayList<>();
        double t = 0;
        for (CubicBezierCurve curve : bezierCurves) {
            for (Vector2 pos : curve.getPoints()) {
                points.add(new Point(pos, 0.0, t));
                t += 1.0 / Tuning.CURVE_RESOLUTION;
            }
        }
        return points;
    }

    private static List<ActionPoint> processActionPoints(JsonNode actionPointsNode) {
        List<ActionPoint> actionPoints = new ArrayList<>();
        for (JsonNode node : actionPointsNode) {
            actionPoints.add(new ActionPoint(
                node.get("t").asDouble(),
                node.get("isStopped").asBoolean(),
                node.get("rotationRadians").asDouble()
            ));
        }
        actionPoints.sort(Comparator.comparingDouble(ActionPoint::t));
        return actionPoints;
    }

    private static void assignRotations(List<Point> points, List<ActionPoint> actionPoints) {
        if(actionPoints.isEmpty()) return;


        int actionIndex = 0;
        for (Point point : points) {
            while (actionIndex < actionPoints.size() - 1 && point.getT() > actionPoints.get(actionIndex).t()) {
                actionIndex++;
            }
            point.setRotation(actionPoints.get(actionIndex).rotation());
        }
    }

    private static Vector2 parseControlPoint(JsonNode node) {
        return new Vector2(
            node.get("x").asDouble() - (Constants.FIELD_WIDTH/2),
            -((Constants.FIELD_HEIGHT/2) -node.get("y").asDouble())
        );
    }

    private static NavigableMap<Double, Point> mapPoints(List<Point> points) {
        NavigableMap<Double, Point> map = new TreeMap<>();
        for (Point point : points) {
            map.put(point.getT(), point);
        }
        return map;
    }
}
