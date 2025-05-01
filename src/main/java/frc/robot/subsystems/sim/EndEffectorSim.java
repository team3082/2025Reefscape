package frc.robot.subsystems.sim;

import frc.robot.utils.PIDController;
import frc.robot.utils.RTime;

/*
 * Simulates the end effector, uses simple PID control to move to a target position, and ramp up to a target wheel speed
 * this is read from during sim instead of real motor values
 */
public class EndEffectorSim {

    private static PIDController posPID = new PIDController(7.5, 0, 0, 0, 0, 12.5);

    private static double targetPos, targetSpeed; // Radians, PercentOut
    private static double pos, speed; // Radians, PercentOut

    private static final double MAX_RAMP = 2.0; // percent out / second

    /**
     * updates position (via pid) and speed
     */
    public static void update() {
        // update pos
        pos += posPID.updateOutput(pos) * RTime.deltaTime();

        // update speed
        double speedError = speed - targetSpeed;
        if (Math.abs(speedError) <= (MAX_RAMP * RTime.deltaTime())) speed = targetSpeed;
        else {
            if (speedError < 0) speed += MAX_RAMP * RTime.deltaTime();
            else speed -= MAX_RAMP * RTime.deltaTime();
        }
    }

    /**
     * sets the target/desired position of the endeffector
     * @param setPos position to set to
     */
    public static void setPosition(double setPos) {
        if (targetPos != setPos) {
            targetPos = setPos;
            posPID.setDest(targetPos);
        }
    }

    /**
     * sets the target/desired speed of the endeffector
     * @param setSpeed speed to set to
     */
    public static void setSpeed(double setSpeed) {
        targetSpeed = setSpeed;
    }

    /**
     * returns the position
     * @return position
     */
    public static double getPosition() {
        return pos;
    }

    /**
     * returns the speed
     * @return speed
     */
    public static double getSpeed() {
        return speed;
    }
}
