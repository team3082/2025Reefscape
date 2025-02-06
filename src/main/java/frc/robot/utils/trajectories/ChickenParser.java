package frc.robot.utils.trajectories;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import frc.robot.Constants;
import frc.robot.utils.Vector2;

public class ChickenParser {
    String pathJSON;
    List<RobotPath> paths;

    public ChickenParser(String pathJSON) {
        this.pathJSON = pathJSON;
        loadPath();
    }

    public void loadPath() {
        // load json and read bezier curve data to generate new bezier curve class
        ObjectMapper mapper = new ObjectMapper();
        
        try {
            JsonNode root = mapper.readTree(new File(pathJSON));

            JsonNode controlPoints = root.get("controlPoints");
            JsonNode actionPoints = root.get("actionPoints");

            int numCurves = (controlPoints.size() - 1) / 3;
            List<BezierCurve> bezierCurves = new ArrayList<>();
            for (int i = 0; i < numCurves; i++) {
                JsonNode a = controlPoints.get((i * 3));
                JsonNode b = controlPoints.get((i * 3) + 1);
                JsonNode c = controlPoints.get((i * 3) + 2);
                JsonNode d = controlPoints.get((i * 3) + 3);
                bezierCurves.add(new BezierCurve (
                    new Vector2((a.get("y").asDouble() * Constants.METERSTOINCHES - (Constants.FIELD_HEIGHT / 2.0)), -1 * (a.get("x").asDouble() * Constants.METERSTOINCHES - (Constants.FIELD_WIDTH / 2.0))),
                    new Vector2((b.get("y").asDouble() * Constants.METERSTOINCHES - (Constants.FIELD_HEIGHT / 2.0)), -1 * (b.get("x").asDouble() * Constants.METERSTOINCHES - (Constants.FIELD_WIDTH / 2.0))),
                    new Vector2((c.get("y").asDouble() * Constants.METERSTOINCHES - (Constants.FIELD_HEIGHT / 2.0)), -1 * (c.get("x").asDouble() * Constants.METERSTOINCHES - (Constants.FIELD_WIDTH / 2.0))),
                    new Vector2((d.get("y").asDouble() * Constants.METERSTOINCHES - (Constants.FIELD_HEIGHT / 2.0)), -1 * (d.get("x").asDouble() * Constants.METERSTOINCHES - (Constants.FIELD_WIDTH / 2.0)))
                ));
            }

            List<Vector2> curvePointsList = new ArrayList<>();
            for (BezierCurve curve : bezierCurves) {
                curvePointsList.addAll(curve.getPoints());
            }

            // generate list of all stop points in the path
            List<Double> stopPointsList = new ArrayList<>();
            List<RotPoint> rotPointsList = new ArrayList<>();
            for (JsonNode actionPoint : actionPoints) {
                if (actionPoint.get("isStopped").asBoolean() == true) {
                    stopPointsList.add(actionPoint.get("t").asDouble());
                }
                rotPointsList.add(new RotPoint(actionPoint.get("rotationRadians").asDouble(), actionPoint.get("t").asDouble()));
            }

            // generate total path and seperate into individual paths
            RobotPath totalPath = new RobotPath(curvePointsList, rotPointsList);
            paths = totalPath.separatePaths(stopPointsList);
            
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public List<RobotPath> getPaths() {
        return paths;
    }
}
