package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
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
        movePID = new PIDController(2.0, 0.1, 0.15, 0.01, 0.00, 1.0);
        movePID.setDest(1.0);
        this.path = path;
        System.out.println("end t" + this.path.getLastPos());
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
        SwerveManager.update();
    }

    @Override
    public void end(boolean interrupted) {
        SwerveManager.rotateAndDrive(0, new Vector2());
        SwerveManager.update();
        if (Robot.isSimulation()) {
            SwerveManager.mods[0].simModule.speed = 0;
            SwerveManager.mods[1].simModule.speed = 0;
            SwerveManager.mods[2].simModule.speed = 0;
            SwerveManager.mods[3].simModule.speed = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
