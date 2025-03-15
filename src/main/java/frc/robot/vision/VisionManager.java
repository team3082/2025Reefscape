package frc.robot.vision;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.Vector2;

public class VisionManager {

    private static Camera[] cameras;
    private static boolean enabled;

    public static void init(){

        if (Robot.isReal()) {
            cameras = new Camera[] {
                // new Camera(new PhotonCamera("ApriltagCamera3"), new Vector2(-9.75, 9), 0.0, Math.toRadians(-15)), // test these later
                new Camera(new PhotonCamera("Global_Shutter_Camera"), new Vector2(10, 10), 0, Math.toRadians(15))
            };
        }

    }

    public static Optional<Vector2> getPosition(double pigeonAngle) {

        List<Vector2> positions = new ArrayList<>();

        for (Camera camera : cameras) {
            
            PhotonTrackedTarget target = camera.photonCamera.getLatestResult().getBestTarget();
            
            System.out.println();
            
            if (target != null) if (camera.isLatestTarget(target)) {
                continue;
            }

            camera.setLatestTarget(target);
            if (target == null) continue; // Skip if no april tags are found

            Transform3d transform = target.getBestCameraToTarget();
            int id = target.getFiducialId();

            if (id < 0 || id > Constants.APRIL_TAGS.length) {
                continue; // Skip invalid id
            }
            
            Vector2 vectorTransform = new Vector2(transform.getX(), transform.getY());
            vectorTransform = vectorTransform.rotate(camera.cameraYaw);

            // Rotate robot position to align with field coordinate frame
            double xdistRobot = vectorTransform.x * Math.cos(camera.cameraPitch) - transform.getZ() * Math.sin(camera.cameraPitch);
            double ydistRobot = vectorTransform.y;

            Vector2 distRobot = new Vector2(xdistRobot, ydistRobot);
            if(distRobot.mag() < 0 || distRobot.mag() > 3){
                continue;
            }

            double xdistField = (Math.cos(pigeonAngle) * distRobot.x - Math.sin(pigeonAngle) * distRobot.y) * Constants.METERSTOINCHES;
            double ydistField = (Math.cos(pigeonAngle) * distRobot.y + Math.sin(pigeonAngle) * distRobot.x) * Constants.METERSTOINCHES;

            Vector2 cameraToTag = new Vector2(xdistField, ydistField);



            Vector2 aprilTagPos = new Vector2(Constants.APRIL_TAGS[id].getPosition().y, -Constants.APRIL_TAGS[id].getPosition().x);
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                aprilTagPos = aprilTagPos.rotate(Math.PI);
            }
            
            Vector2 cameraPos = aprilTagPos.sub(cameraToTag);

            Vector2 robotPos = cameraPos.sub(camera.robotToCamera.rotate(pigeonAngle - (Math.PI/2.0)));

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

    public static void enableVision(){
        enabled = true;
    }
    
    public static void disableVision(){
        enabled = false;
    }

    public static boolean isEnabled(){
        return enabled;
    }
    
}
