package frc.robot.subsystems.sim;

import frc.robot.utils.PIDController;
import frc.robot.utils.RTime;

public class AlgaeSim {

    private static PIDController anglePID = new PIDController(6, 0, 0.5, 0, 0, 50.0);

    private static double angle;
    private static double targetAngle;
    private static double speed;
    private static double targetSpeed;

    private static final double MAX_RAMP = 100.0;

    public static void update(){
        angle += anglePID.updateOutput(angle) * RTime.deltaTime();

        double speedError = speed - targetSpeed;
        if (Math.abs(speedError) <= (MAX_RAMP * RTime.deltaTime())) speed = targetSpeed;
        else {
            if (speedError < 0) speed += MAX_RAMP * RTime.deltaTime();
            else speed -= MAX_RAMP * RTime.deltaTime();
        }

    }

    public static void setAngle(double setAngle){
        if(setAngle != targetAngle){
            targetAngle = setAngle;
            anglePID.setDest(targetAngle);
        }
    }

    public static void setSpeed(double setAngle){
        targetSpeed = setAngle;

    }

    public static double getAngle(){
        return angle;
    }

    

    public static double getSpeed(){
        return speed;
    }

}
