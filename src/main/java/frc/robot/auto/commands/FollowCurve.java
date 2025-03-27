package frc.robot.auto.commands;

import frc.robot.Tuning;
import frc.robot.utils.Vector2;
import frc.robot.utils.PIDController;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.trajectories.Curve;
import frc.robot.subsystems.sensors.Pigeon;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.trajectories.RobotPath;

public class FollowCurve extends Command {
    private RobotPath path;
    private boolean isFinished = false;

    private PIDController movePID;
    private PIDController rotPID;

    double maxSpeed = 1.0;

    public FollowCurve(Curve curve, double targetRot, double maxVelMove, double maxVelRot) {
        this.path = new RobotPath(curve.getPoints(), targetRot);
        this.movePID = new PIDController(1.75, 0.025, 0.1575, 0.0, 0.0, maxVelMove);
        this.rotPID = new PIDController(0.35, 0.015, 0.08, 0.035, 0.1, maxVelMove);
    }

    public FollowCurve(Curve curve, double targetRot, double maxVelMove, double maxVelRot, double rotDead) {
        this.path = new RobotPath(curve.getPoints(), targetRot);
        this.movePID = new PIDController(1.75, 0.025, 0.1575, 0.0, 0.0, maxVelMove);
        this.rotPID = new PIDController(0.35, 0.015, 0.08, rotDead, 0.1, maxVelMove);
    }

    @Override
    public void initialize() {
        System.out.println("FollowCurve Initialized");
        this.movePID.setDest(1);
        this.rotPID.setDest((path.getTargetRot() + Math.PI / 2.0) % (2.0 * Math.PI));
    }

    @Override
    public void execute() {
        path.updatePosition(SwervePosition.getPosition());
        double currentRot = Pigeon.getRotationRad() % (2.0 * Math.PI);
        if (currentRot - rotPID.getDest() > Math.PI) {
            currentRot -= 2.0 * Math.PI;
        } else if (currentRot - rotPID.getDest() < -Math.PI) {
            currentRot += 2.0 * Math.PI;
        }

        double moveOutput = movePID.updateOutput((path.getPathLength() - path.getRemainingPathLength()) / path.getPathLength());
        double rotOutput = rotPID.updateOutput(currentRot);

        Vector2 driveVector = path.getDriveVector().norm().mul(moveOutput);
        driveVector = driveVector.rotate(-Math.PI / 2.0);

        if (driveVector.mag() > maxSpeed) {
            driveVector = driveVector.norm().mul(maxSpeed);
        }

        SwerveManager.rotateAndDrive(rotOutput, driveVector);

        boolean driveFinished = (SwervePosition.getPosition().sub(path.getLastPos()).mag() < Tuning.CURVE_DEADBAND);
        if (driveFinished) driveVector = new Vector2();
        boolean rotFinished = rotPID.atSetpoint();
        if (rotFinished) rotOutput = 0.0;

        if (driveFinished && rotFinished) {
            isFinished = true;
            end(false);
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        SwerveManager.rotateAndDrive(0.0, new Vector2());
        System.out.println("FollowCurve ended");
    }

    public double getRemainingPathLength() {
        return path.getRemainingPathLength();
    }

    public void setMaxSpeed(double speed) {
        this.maxSpeed = speed;
    }
}
