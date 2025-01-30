package frc.robot.subsystems.sim;

import frc.robot.utils.PIDController;
import frc.robot.utils.RTime;

/*
 * Simulates the elevator, uses simple PID control to move to a target position
 * this is read from during sim instead of real motor values
 */
public class ElevatorSim {

    private static PIDController posPID = new PIDController(50, 0, 0, 0, 0, 50.0);

    private static double targetPos; // Inches
    private static double pos; // Inches

    public static void update() {
        // update pos
        pos += posPID.updateOutput(pos) * RTime.deltaTime();
    }

    public static void setPosition(double setPos) {
        if (targetPos != setPos) {
            targetPos = setPos;
            posPID.setDest(targetPos);
        }
    }

    public static double getPosition() {
        return pos;
    }
}
