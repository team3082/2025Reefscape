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

    /**
     * updates the angle (via pid) and the speed
     */
    public static void update(){
        angle += anglePID.updateOutput(angle) * RTime.deltaTime();

        double speedError = speed - targetSpeed;
        if (Math.abs(speedError) <= (MAX_RAMP * RTime.deltaTime())) speed = targetSpeed;
        else {
            if (speedError < 0) speed += MAX_RAMP * RTime.deltaTime();
            else speed -= MAX_RAMP * RTime.deltaTime();
        }

    }

    /**
     * sets a target/desired angle
     * @param setAngle angle to set to
     */
    public static void setAngle(double setAngle){
        if(setAngle != targetAngle){
            targetAngle = setAngle;
            anglePID.setDest(targetAngle);
        }
    }

    /**
     * sets a target/desired speed
     * @param setSpeed speed to set to
     */
    public static void setSpeed(double setSpeed){
        targetSpeed = setSpeed;

    }

    /**
     * gets the angle
     * @return angle
     */
    public static double getAngle(){
        return angle;
    }    

    /**
     * gets the speed
     * @return speed
     */
    public static double getSpeed(){
        return speed;
    }

}
