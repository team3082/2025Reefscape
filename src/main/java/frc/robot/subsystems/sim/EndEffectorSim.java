package frc.robot.subsystems.sim;

import frc.robot.utils.PIDController;
import frc.robot.utils.RTime;

public class EndEffectorSim {

    private static PIDController posPID = new PIDController(3, 0, 0, 0, 0, 3.0);

    private static double targetPos, targetSpeed; // Radians, PercentOut
    private static double pos, speed; // Radians, PercentOut

    private static final double MAX_RAMP = 2.0; // percent out / second

    public static void update() {
        // update pos
        System.out.println("End Effector Sim Pos: " + pos);
        pos += posPID.updateOutput(pos) * RTime.deltaTime();

        // update speed
        double speedError = speed - targetSpeed;
        if (Math.abs(speedError) <= (MAX_RAMP * RTime.deltaTime())) speed = targetSpeed;
        else {
            if (speedError < 0) speed += MAX_RAMP * RTime.deltaTime();
            else speed -= MAX_RAMP * RTime.deltaTime();
        }
    }

    public static void setPosition(double setPos) {
        System.out.println("End Effector Sim Set Pos: " + setPos);
        if (targetPos != setPos) {
            targetPos = setPos;
            posPID.setDest(targetPos);
        }
    }

    public static void setSpeed(double setSpeed) {
        targetSpeed = setSpeed;
    }

    public static double getPosition() {
        return pos;
    }

    public static double getSpeed() {
        return speed;
    }
}
