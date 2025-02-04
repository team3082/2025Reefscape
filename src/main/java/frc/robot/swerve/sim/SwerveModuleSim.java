package frc.robot.swerve.sim;

import frc.robot.utils.PIDController;
import frc.robot.utils.RTime;

public class SwerveModuleSim {
    private PIDController anglePID = new PIDController(7.5, 0, 0, 0, 0, 7.5);

    private double targetAngle, targetSpeed; // Radians, PercentOut
    private double angle, speed; // Radians, PercentOut

    private final double MAX_RAMP = 2.0; // percent out / second

    public SwerveModuleSim() {}

    public void update() {
        // update pos
        angle += anglePID.updateOutput(angle) * RTime.deltaTime();

        // update speed
        double speedError = speed - targetSpeed;
        if (Math.abs(speedError) <= (MAX_RAMP * RTime.deltaTime())) speed = targetSpeed;
        else {
            if (speedError < 0) speed += MAX_RAMP * RTime.deltaTime();
            else speed -= MAX_RAMP * RTime.deltaTime();
        }
    }

    public void setAngle(double setPos) {
        if (targetAngle != setPos) {
            targetAngle = setPos;
            anglePID.setDest(targetAngle);
        }
    }

    public void setSpeed(double setSpeed) {
        targetSpeed = setSpeed;
    }

    public double getAngle() {
        return angle;
    }

    public double getSpeed() {
        return speed;
    }
}
