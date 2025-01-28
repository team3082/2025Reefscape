package frc.robot.swerve.sim;

import frc.robot.utils.PIDController;

public class SwerveModuleSim {

    private PIDController posPID = new PIDController(0, 0, 0, 0, 0, 0);

    private double targetRot, targetDrive;
    private double rot, drive;
    private double x, y;

    public SwerveModuleSim(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void update(double targetRot, double targetDrive) {
        // TODO
    }
}
