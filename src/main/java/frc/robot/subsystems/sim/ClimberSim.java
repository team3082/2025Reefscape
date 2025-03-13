package frc.robot.subsystems.sim;

import frc.robot.Tuning;
import frc.robot.utils.PIDController;
import frc.robot.utils.RTime;

/*
 * Simulates the climber, uses a PID controller to simulate the rotation to a target angle.
 */
public class ClimberSim {

    private static PIDController anglePID = new PIDController(
        Tuning.Climber.CLIMBER_P, 
        Tuning.Climber.CLIMBER_I, 
        Tuning.Climber.CLIMBER_D,
        0, 0, 0.0
    );

    private static double targetAngleRaidans;
    private static double angleRadians;

    /**
     * Updates the angle of the climber based off the set position.
     */
    public static void update() {
        angleRadians += anglePID.updateOutput(angleRadians) * RTime.deltaTime();
    }

    /**
     * Sets the angle of the climber to a desired angle.
     * 
     * @param setAngle the angle to set the climber to.
     */
    public static void setAngle(double setAngle) {
        if (targetAngleRaidans != setAngle) {
            targetAngleRaidans = setAngle;
            anglePID.setDest(targetAngleRaidans);
        }
    }

    /**
     * @return returns the current angle of the climber.W
     */
    public static double getAngle() {
        return angleRadians;
    }
}