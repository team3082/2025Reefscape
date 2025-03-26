package frc.robot.swerve;

import static frc.robot.Tuning.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.utils.PIDController;
import frc.robot.utils.RotationalPIDController;
import frc.robot.utils.Vector2;

public class SwervePID {

    public static PIDController movePID, rotPID;
    private static Vector2 startPos;
    private static Vector2 moveDest = new Vector2();

    private static double totalDist;

    public static void init() {
        movePID = new PIDController(MOVEP, MOVEI, MOVED, 0.0, 0.01, MOVEMAXSPEED);
        rotPID = new RotationalPIDController(ROTP, ROTI, ROTD, ROTDEAD, 0.01, ROTMAXSPEED);
    }

    public static void setDestState(Vector2 dest, double destRot) {
        try {
            Logger.recordOutput("Robot/SwervePID/Destination", new Pose2d(dest.convertToFieldCoords().x,
                                                                              dest.convertToFieldCoords().y,
                                                                              Rotation2d.fromRadians(destRot + Math.PI/2)));
        } catch (Exception e) {}
        moveDest = dest;
        startPos = SwervePosition.getPosition();
        totalDist = dest.sub(startPos).mag();
        movePID.setDest(totalDist);
        rotPID.setDest(destRot);
    }
    
    public static double updateOutputRot() {
        return rotPID.updateOutput(Pigeon.getRotationRad());
    }

    public static Vector2 updateOutputVel() {
        Vector2 currentPos = SwervePosition.getPosition();
        Vector2 driveVector = moveDest.sub(currentPos).norm();
        double currentDist = moveDest.sub(currentPos).mag();
        double driveOutput = movePID.updateOutput((totalDist - currentDist));
        driveVector = driveVector.mul(driveOutput);

        if (atDest()) driveVector = new Vector2(0, 0);

        return driveVector.rotate(-Math.PI/2);
    }

    public static Vector2 getDest() {
        return moveDest;
    }

    public static double getTargetRot(){
        return rotPID.getDest();
    }

    public static boolean atDest() {
        return moveDest.sub(SwervePosition.getPosition()).mag() < MOVEDEAD;
    }
    
    public static boolean atRot() {
        return rotPID.atSetpoint();
    }

    public static Vector2 getError() {
        return moveDest.sub(SwervePosition.getPosition());
    }

    public static double getRotationError() {
        return rotPID.getError();
    }

}