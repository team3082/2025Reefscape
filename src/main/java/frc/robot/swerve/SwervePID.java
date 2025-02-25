package frc.robot.swerve;

import static frc.robot.Tuning.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.utils.PIDController;
import frc.robot.utils.RMath;
import frc.robot.utils.RotationalPIDController;
import frc.robot.utils.Vector2;

public class SwervePID {

    public static PIDController xPID, yPID, rotPID;

    public static double moveP = MOVEP;
    public static double moveI = MOVEI;
    public static double moveD = MOVED;
    public static double moveDead = MOVEDEAD;
    public static double moveVelDead = MOVEVELDEAD;
    public static double moveSpeedMax = MOVEMAXSPEED;

    public static double rotP = ROTP;
    public static double rotI = ROTI;
    public static double rotD = ROTD;
    public static double rotDead = ROTDEAD;
    public static double rotVelDead = ROTVELDEAD;
    public static double rotSpeedMax = ROTMAXSPEED;

    public static void init() {
        xPID = new PIDController(moveP, moveI, moveD, moveDead, moveVelDead, moveSpeedMax);
        yPID = new PIDController(moveP, moveI, moveD, moveDead, moveVelDead, moveSpeedMax);
        rotPID = new RotationalPIDController(rotP, rotI, rotD, rotDead, rotVelDead, rotSpeedMax);
    }

    public static void setDestState(Vector2 dest, double destRot) {
        setDestPt(dest);
        setDestRot(destRot);
    }

    public static void setDestPt(Vector2 dest) {
        setDestX(dest.x);
        setDestY(dest.y);
    }

    public static void setDestX(double dest) {
        xPID.setDest(dest);
    }

    public static void setDestY(double dest) {
        yPID.setDest(dest);
    }

    public static void setDestRot(double dest) {
        rotPID.setDest(RMath.targetAngleAbsolute(Pigeon.getRotationRad(), dest, 2*Math.PI));
    }
    
    public static double updateOutputX() {
        return (xPID.atSetpoint() ? 0 : (DriverStation.getAlliance().get() == Alliance.Red ? 1 : 1) * xPID.updateOutput(SwervePosition.getPosition().x));
    }

    public static double updateOutputY() {
        return yPID.atSetpoint() ? 0 : yPID.updateOutput(SwervePosition.getPosition().y);
    }

    public static double updateOutputRot() {
        return rotPID.updateOutput(Pigeon.getRotationRad());
    }

    public static Vector2 updateOutputVel() {
        return new Vector2(updateOutputX(), updateOutputY()).rotate((DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1) * Math.PI/2);
    }

    public static Vector2 getDest() {
        return new Vector2(xPID.getDest(), yPID.getDest());
    }

    public static double getTargetRot(){
        return rotPID.getDest();
    }

    public static boolean atDest() {
        return xPID.atSetpoint() && yPID.atSetpoint();
    }
    
    public static boolean atRot() {
        return rotPID.atSetpoint();
    }

    public static Vector2 getError() {
        return new Vector2(xPID.getError(), yPID.getError());
    }

    public static double getRotationError() {
        return rotPID.getError();
    }

}