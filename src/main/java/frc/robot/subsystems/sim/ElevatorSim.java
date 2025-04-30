package frc.robot.subsystems.sim;

import frc.robot.utils.PIDController;
import frc.robot.utils.RTime;

/*
 * Simulates the elevator, uses simple PID control to move to a target position
 * this is read from during sim instead of real motor values
 */
public class ElevatorSim {

    // max speeds - 100 to sim 15:1 ratio - 75 to sim 25:1 ratio
    private static PIDController posPID = new PIDController(50, 0, 0, 0, 0, 100);

    private static double targetPos; // Inches
    private static double pos; // Inches

    /**
     * updates the position via pid
     */
    public static void update() {
        pos += posPID.updateOutput(pos) * RTime.deltaTime();
    }

    /**
     * sets the target/desired position of the elevator
     * @param setPos position to set to
     */
    public static void setPosition(double setPos) {
        if (targetPos != setPos) {
            targetPos = setPos;
            posPID.setDest(targetPos);
        }
    }

    /**
     * returns position of the elevator
     * @return position
     */
    public static double getPosition() {
        return pos;
    }
}
