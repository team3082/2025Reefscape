package frc.robot.subsystems.sim;

import frc.robot.Tuning;
import frc.robot.utils.PIDController;
import frc.robot.utils.RTime;

/*
 * Simulates the climber, uses simple PID control to rotate to a target angle
 * this is read from during sim instead of real motor values
 */
public class ClimberSim {

    private static PIDController anglePID = new PIDController(Tuning.Climber.CLIMBER_P, Tuning.Climber.CLIMBER_I,
            Tuning.Climber.CLIMBER_D, 0, 0, 50.0);

    private static double targetAngle; // radians
    private static double angle; // radians

    /**
     * updates the angle of the climber
     */
    public static void update() {
        angle += anglePID.updateOutput(angle) * RTime.deltaTime();
    }

    /**
     * sets the angle of the climber to a desired angle
     * 
     * @param setAngle the angle to set the climber to
     */
    public static void setAngle(double setAngle) {
        if (targetAngle != setAngle) {
            targetAngle = setAngle;
            anglePID.setDest(targetAngle);
        }
    }

    /**
     * @return returns the current angle of the climber
     */
    public static double getAngle() {
        return angle;
    }
}