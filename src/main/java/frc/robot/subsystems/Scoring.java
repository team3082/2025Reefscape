package frc.robot.subsystems;

import frc.robot.utils.PIDController;
import frc.robot.utils.Vector2;
import frc.robot.vision.AprilTag;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePosition;

public class Scoring {
    public enum ScoringAlignment {
        LEFT,
        CENTER,
        RIGHT
    }
    public enum ScoringState {
        DRIVER_CONTROL,
        ALIGNING_TO_REEF,
        EXTENDING,
        SCORING,
        RETRACTING
    }

    public static Vector2 targetPos;
    public static double totalDist;
    public static double targetRot;

    public static ScoringState currentState;
    public static ScoringAlignment targetSide;

    public static PIDController movePID = new PIDController(0, 0, 0, 0, 0, 0);
    public static PIDController rotPID = new PIDController(0, 0, 0, 0, 0, 0);

    public static void update() {
        switch (currentState) {
            case DRIVER_CONTROL:
                ScoringManager.setScoringPosition(ScoringManager.ScoringPosition.STOW);

                Vector2 driveVector = new Vector2(OI.driverStick.getRawAxis(OI.moveX), -OI.driverStick.getRawAxis(OI.moveY));
                double rot = OI.driverStick.getRawAxis(OI.rotateX) * 0.5; // rot max coefficient

                SwerveManager.rotateAndDrive(rot, driveVector);
                break;

            case ALIGNING_TO_REEF:
                Vector2 currentPos = SwervePosition.getPosition();
                double moveError = targetPos.sub(currentPos).mag();
                double driveMag = Math.abs(movePID.updateOutput(moveError));
                Vector2 moveVector = targetPos.sub(currentPos).norm().mul(driveMag);
                
                double currentRot = Pigeon.getRotationRad();
                double rotMag = rotPID.updateOutput(currentRot );

                SwerveManager.rotateAndDrive(rotMag, moveVector);

                if (movePID.atSetpoint() && rotPID.atSetpoint()) {
                    currentState = ScoringState.EXTENDING;
                }
                break;

            case EXTENDING:
                ScoringManager.setScoringPosition(OI.savedLevel);
                if (ScoringManager.transitoryState == ScoringManager.TransitoryState.FINISHED) currentState = ScoringState.SCORING;
                break;

            case SCORING:
                ScoringManager.getEndEffector().setIntakeState(EndEffector.IntakeState.DROP_CORAL);
                if (ScoringManager.getEndEffector().isHoldingCoral()) currentState = ScoringState.RETRACTING;
                break;

            case RETRACTING:
                ScoringManager.setScoringPosition(ScoringManager.ScoringPosition.STOW);
                currentState = ScoringState.DRIVER_CONTROL;
                break;
        }
    }

    public static void enableDriverControl() {
        currentState = ScoringState.DRIVER_CONTROL;
    }

    public static void alignToReef(ScoringAlignment side) {
        targetSide = side;
        Vector2 startingPos = SwervePosition.getPosition();
        AprilTag targetTag = getClosestNode(startingPos);
        switch (side) {
            case LEFT:
                targetPos = targetTag.getLeftPosition();
                break;
            case CENTER:
                targetPos = targetTag.getCenterPosition();
                break;
            case RIGHT:
                targetPos = targetTag.getRightPosition();
                break;
        }
        totalDist = targetPos.sub(startingPos).mag();
        movePID.setDest(0);
        rotPID.setDest(targetTag.getRotationZ());
        currentState = ScoringState.ALIGNING_TO_REEF;
    }

    public static AprilTag[] reefTags = {Constants.APRIL_TAGS[6], Constants.APRIL_TAGS[7], Constants.APRIL_TAGS[8], Constants.APRIL_TAGS[9], Constants.APRIL_TAGS[10], Constants.APRIL_TAGS[11]};
    
    public static AprilTag getClosestNode(Vector2 currentPos) {
        AprilTag closestTag = null;
        switch (targetSide) {
            case LEFT:
                for (AprilTag tag : reefTags) {
                    if (closestTag == null || currentPos.sub(tag.getLeftPosition()).mag() < currentPos.sub(closestTag.getLeftPosition()).mag()) {
                        closestTag = tag;
                    }
                }
                break;

            case CENTER:
                for (AprilTag tag : reefTags) {
                    if (closestTag == null || currentPos.sub(tag.getCenterPosition()).mag() < currentPos.sub(closestTag.getCenterPosition()).mag()) {
                        closestTag = tag;
                    }
                }
                break;
            case RIGHT:
                for (AprilTag tag : reefTags) {
                    if (closestTag == null || currentPos.sub(tag.getRightPosition()).mag() < currentPos.sub(closestTag.getRightPosition()).mag()) {
                        closestTag = tag;
                    }
                }
                break;
        }
        return closestTag;
    }
}
