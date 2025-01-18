package frc.robot.swerve;

import static frc.robot.Tuning.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
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
        if(!Robot.isSimulation()){
            return (xPID.atSetpoint() ? 0 : (DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1) * xPID.updateOutput(SwervePosition.getPosition().x));
        } else {
            return getDest().sub(SwervePosition.getPosition()).norm().mul(.5).x;
        }
    }

    public static double updateOutputY() {
        if(!Robot.isSimulation())
            return yPID.atSetpoint()? 0 : yPID.updateOutput(SwervePosition.getPosition().y);
        else{
            return getDest().sub(SwervePosition.getPosition()).norm().mul(.5).y; //.0085
        }
    }

    public static double updateOutputRot() {
        if(RobotBase.isReal())
            return rotPID.updateOutput(Pigeon.getRotationRad());
        else{
            Pigeon.setSimulatedRot(rotPID.getDest());
            return rotPID.updateOutput(Pigeon.getRotationRad());
        }
    }

    public static Vector2 updateOutputVel() {
        return new Vector2(updateOutputX(), updateOutputY());
    }

    public static Vector2 getDest() {
        return new Vector2(xPID.getDest(), yPID.getDest());
    }

    public static boolean atDest() {

        
        return (Robot.isSimulation()
        && SwervePosition.getPosition().isGreater(new Vector2(getDest().x - moveDead, getDest().y - moveDead)) 
        && !SwervePosition.getPosition().isGreater(new Vector2(getDest().x + moveDead, getDest().y + moveDead))
        )||(xPID.atSetpoint() && yPID.atSetpoint());
    }
    
    public static boolean atRot() {
        return (Robot.isSimulation() && Pigeon.simulatedRot == rotPID.getDest()) || rotPID.atSetpoint();
    }

}