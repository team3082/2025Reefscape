package frc.robot.subsystems.sim;

import frc.robot.utils.PIDController;
import frc.robot.utils.RTime;

public class ElevatorSim {

    private static PIDController posPID = new PIDController(0, 0, 0, 0, 0, 0);

    private static double targetPos; // Inches
    private static double pos; // Inches

    public static void update() {
        // update pos
        pos += posPID.updateOutput(pos) * RTime.deltaTime();
    }

    public static void setPosition(double setPos) {
        targetPos = setPos;
    }

    public static double getPosition() {
        return pos;
    }
}
