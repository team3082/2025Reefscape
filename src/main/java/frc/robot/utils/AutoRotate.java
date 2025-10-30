package frc.robot.utils;

import frc.robot.Constants;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwerveManager;

public class AutoRotate {

    static double rightCoralStationAngle = Math.toRadians(145);
    static double leftCoralStationAngle = Math.toRadians(35);
    static Vector2 reefCenter = new Vector2(170, 0);

    public static void init() {

    }

    public static double getRotation(Vector2 currentPos) {
        double targetRot = -Math.PI / 2;
        double robotRot = RMath.modulo(Pigeon.getRotationRad(), 2.0 * Math.PI);

        if (inRightCoralStationZone(currentPos)) {
            targetRot = rightCoralStationAngle;
        } else if (inLeftCoralStationZone(currentPos)) {
            targetRot = leftCoralStationAngle;
        } else if (inReefZone(currentPos)) {
            double centerToRobotAngle = currentPos.sub(reefCenter).atan2();
            targetRot = Math.PI/2 + centerToRobotAngle;        
        }

        targetRot = RMath.modulo(targetRot, 2.0 * Math.PI);
        if (targetRot - robotRot > Math.PI) {
            robotRot += 2.0 * Math.PI;
        } else if (robotRot - targetRot > Math.PI) {
            robotRot -= 2.0 * Math.PI;
        }

        return targetRot - robotRot;
    }

    public static double getShortestRotation(double rotation, double robotRotation) {
        double targetAngle = RMath.modulo(robotRotation + rotation, 2.0 * Math.PI);
        return targetAngle;
    }

    public static Boolean inRightCoralStationZone(Vector2 currentPos) {
        if (currentPos.y + 0.7 * currentPos.x > 300) {
            return true;
        }
        return false;
    }

    public static Boolean inLeftCoralStationZone(Vector2 currentPos) {
        if (currentPos.y - 0.7 * currentPos.x < -300) {
            return true;
        }
        return false;
    }

    public static Boolean inReefZone(Vector2 currentPos) {
        return currentPos.x > 60;
    }
}
