package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Tuning;

import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.trajectories.RobotPath;
import frc.robot.utils.PIDController;
import frc.robot.utils.Vector2;

public class FollowRobotPath extends Command {
    RobotPath path;

    PIDController movePID;

    boolean isFinished = false;

    public FollowRobotPath(RobotPath path) {
        System.out.println("new FollowRobotPath Initialized");
        movePID = new PIDController(Tuning.MOVEP, Tuning.MOVEI, Tuning.MOVED, 0.01, Tuning.MOVEVELDEAD, 0.5);
        movePID.setDest(1.0);
        this.path = path;
    }

    @Override
    public void execute() {
        // update PID controller, control swerve
        double output = movePID.updateOutput(((path.getPathLength() - path.getRemainingPathLength())) / path.getPathLength());
        path.updatePosition(SwervePosition.getPosition());
        Vector2 driveVector = path.getDriveVector().mul(output);
        SwerveManager.rotateAndDrive(0, driveVector);

        isFinished = SwervePosition.getPosition().sub(path.getLastPos()).mag() < Tuning.CURVE_DEADBAND; // check if path following is finished

        if (isFinished) {
            System.out.println("Path Finished");
            SwerveManager.rotateAndDrive(0, new Vector2());
            end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        SwerveManager.rotateAndDrive(0, new Vector2());
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
