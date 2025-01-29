package frc.robot.subsystems.sim;

import frc.robot.utils.PIDController;
import frc.robot.utils.RTime;

public class ElevatorSim {

    private static PIDController posPID = new PIDController(20, 0, 0, 0, 0, 20.0);

    private static double targetPos; // Inches
    private static double pos; // Inches

    public static void update() {
        // update pos
        pos += posPID.updateOutput(pos) * RTime.deltaTime();
        System.out.println("Elevator Sim Pos: " + pos);
    }

    public static void setPosition(double setPos) {
        System.out.println("Elevator Sim Set Pos: " + setPos);
        if (targetPos != setPos) {
            System.out.println("Elevator Sim Set Pos: " + setPos);
            targetPos = setPos;
            posPID.setDest(targetPos);
        }
    }

    public static double getPosition() {
        return pos;
    }
}
