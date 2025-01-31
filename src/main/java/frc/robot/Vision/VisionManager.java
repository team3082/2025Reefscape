package frc.robot.vision;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;

import frc.robot.utils.Vector2;

public class VisionManager {

    private static Camera[] cameras;

    public static void init(){

        cameras = new Camera[] {
            new Camera(new PhotonCamera("ApriltagCamera1"), new Vector2(0.0, 0.0), 0.0, 0.0)
        };

    }

    public static Optional<Vector2> getPosition(double pigeonAngle) {

        List<Vector2> positions = new ArrayList<>();

        for (Camera camera : cameras) {
            
            PhotonTrackedTarget target = camera.photonCamera.getLatestResult().getBestTarget();
            if (target == null) continue; // Skip if no april tags are found

            Transform3d transform = target.getBestCameraToTarget();
            int id = target.getFiducialId();

            if (id < 0 || id > Constants.APRIL_TAGS.length) {
                continue; // Skip invalid id
            }

            // Rotate robot position to align with field coordinate frame
            double xdistRobot = transform.getX() * Math.cos(camera.cameraPitch) - transform.getZ() * Math.sin(camera.cameraPitch);
            double ydistRobot = transform.getY();

            double xdistField = xdistRobot * Math.cos(pigeonAngle) - ydistRobot * Math.sin(pigeonAngle);
            double ydistField = ydistRobot * Math.cos(pigeonAngle) + xdistRobot * Math.sin(pigeonAngle);
            
            Vector2 cameraToTag = new Vector2(xdistField, ydistField);

            Vector2 aprilTagPos = Constants.APRIL_TAGS[id - 1].getPosition();
            Vector2 cameraPos = aprilTagPos.sub(cameraToTag);

            Vector2 robotPos = cameraPos.sub(camera.robotToCamera);

            positions.add(robotPos);
        }

        if (positions.isEmpty()) {
            return Optional.empty();
        }

        // Average out robotPos with all camera positions
        double sumX = 0, sumY = 0;
        for (Vector2 position : positions) {
            sumX += position.x;
            sumY += position.y;
        }

        Vector2 averagePosition = new Vector2(sumX / positions.size(), sumY / positions.size());
        return Optional.of(averagePosition);
    };

    public static Optional<Double> getRotation(double pigeonAngle) {

        // Warning! Current solution does not account for cameras having roll, only pitch and yaw

        List<Double> robotYaws = new ArrayList<>();

        for (Camera camera : cameras) {

            PhotonTrackedTarget target = camera.photonCamera.getLatestResult().getBestTarget();

            if (target == null) continue; // Skip if no april tags are found

            // Calculate robot rotation
            Transform3d transform = target.getBestCameraToTarget();
            Rotation3d rotationTransform = transform.getRotation();
            int id = target.getFiducialId();

            double robotToTagYaw = rotationTransform.getZ() + camera.cameraYaw;
            double aprilTagYaw = Constants.APRIL_TAGS[id - 1].getRotationY();

            double robotYaw = robotToTagYaw + aprilTagYaw;

            robotYaws.add(robotYaw);
            
        }

        if (robotYaws.isEmpty()) {
            return Optional.empty();
        }
            
        // Average out robotYaw with other camera positions
        double averageRotation = robotYaws.stream()
            .mapToDouble(Double::doubleValue)
            .average()
            .getAsDouble();

        return Optional.of(averageRotation);
    }
}