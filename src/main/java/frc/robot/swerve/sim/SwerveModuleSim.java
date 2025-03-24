package frc.robot.swerve.sim;

import frc.robot.utils.PIDController;
import frc.robot.utils.RTime;

public class SwerveModuleSim {
    private PIDController anglePID = new PIDController(25.0, 0.025, 0.15, 0, 0, 25.0);

    private double targetAngle, targetSpeed; // Radians, PercentOut
    public double angle, speed; // Radians, PercentOut
    private double drivePos = 0.0;

    private final double MAX_RAMP = 2.0; // percent out / second


    public void update(boolean inverted) {
        // update pos
        angle += anglePID.updateOutput(angle) * RTime.deltaTime();

        // update speed
        double speedError = speed - targetSpeed;
        if (Math.abs(speedError) <= (MAX_RAMP * RTime.deltaTime())) speed = targetSpeed;
        else {
            if (speedError < 0) speed += MAX_RAMP * RTime.deltaTime();
            else speed -= MAX_RAMP * RTime.deltaTime();
        }

        drivePos += speed * RTime.deltaTime() * 200 * (inverted ? -1 : 1);
    }

    public void setAngle(double setPos) {
        setPos = -setPos;
        if (targetAngle != setPos) {
            targetAngle = setPos;
            anglePID.setDest(targetAngle);
        }
    }

    public double getDrivePosition() {
        return drivePos;
    }

    public void setSpeed(double setSpeed) {
        targetSpeed = setSpeed;
    }

    public double getAngle() {
        return -angle;
    }

    public double getSpeed() {
        return speed;
    }
}
