package frc.robot.auto.commands;

import java.util.Vector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.PIDController;
import frc.robot.utils.Vector2;
import frc.robot.utils.trajectories.Curve;
import frc.robot.utils.trajectories.RobotPath;

public class FollowCurve extends Command {
    private RobotPath path;
    private boolean isFinished = false;

    private PIDController movePID;
    private PIDController rotPID;

    public FollowCurve(Curve curve, double targetRot, double maxVelMove, double maxVelRot) {
        this.path = new RobotPath(Tuning.AutoPaths.getAutoPath(curve).getPoints(), targetRot);
        this.movePID = new PIDController(3.0, 0.01, 0.1, 0.0, 0.0, maxVelMove);
        this.rotPID = new PIDController(0.4, 0.01, 0.15, 0.05, 0.1, maxVelMove);
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

        double moveOutput = movePID.updateOutput( (path.getPathLength() - path.getRemainingPathLength()) / path.getPathLength() );
        double rotOutput = rotPID.updateOutput(currentRot);

        Vector2 driveVector = path.getDriveVector().norm().mul(moveOutput);
        if (Robot.isSimulation()) driveVector = driveVector.mul(0.25).rotate(-Math.PI / 2.0);

        System.out.println("rot: " + Pigeon.getRotationRad() + " target rot: " + rotPID.getDest() + " rot output: " + rotOutput);

        // System.out.println("Remaining Path Length: " + path.getRemainingPathLength() + " Path Length: " + path.getPathLength());
        // System.out.println("Drive Vector: " + driveVector + " Move Output: " + moveOutput);

        SwerveManager.rotateAndDrive(rotOutput, driveVector);

        boolean driveFinished = (SwervePosition.getPosition().sub(path.getLastPos()).mag() < Tuning.CURVE_DEADBAND);
        if (driveFinished) driveVector = new Vector2();
        boolean rotFinished = rotPID.atSetpoint();
        if (rotFinished) rotOutput = 0.0;

        // System.out.println("Current Pos: " + SwervePosition.getPosition() + " Target Pos: " + path.getLastPos());
        // System.out.println("Drive Finished: " + driveFinished + " Rot Finished: " + rotFinished);

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
}
