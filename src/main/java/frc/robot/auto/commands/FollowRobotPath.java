package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.OperationDesertStorm;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.trajectories.RobotPath;
import frc.robot.utils.PIDController;
import frc.robot.utils.Vector2;

public class FollowRobotPath extends Command {
    RobotPath path;

    PIDController movePID;
    PIDController rotPID;
    double targetRot;

    boolean isFinished = false;

    public FollowRobotPath(RobotPath path) {
        System.out.println("new FollowRobotPath Initialized");
        this.path = path;
        this.movePID = new PIDController(7.0, 0.075, 0.4, 0.035, 0.00, 0.2);
        this.movePID.setDest(1.0);
        this.rotPID = new PIDController(0.75, 0.0, 0.15, 0.01, 0.0, 0.5);
        targetRot = path.getTargetRot() % (2.0 * Math.PI);

        this.rotPID.setDest(targetRot);
    }

    @Override
    public void execute() {
        // update PID controller, control swerve
        double currentRot = Pigeon.getRotationRad() % (2.0 * Math.PI);
        double output = movePID.updateOutput(((path.getPathLength() - path.getRemainingPathLength())) / path.getPathLength());
        path.updatePosition(SwervePosition.getPosition());
        Vector2 driveVector = path.getDriveVector().mul(output);
        double rotOutput = rotPID.updateOutput(currentRot);

        // System.out.println("Rot: " + currentRot + " Target Rot: " + targetRot + " Rot Output: " + rotOutput);
        System.out.println("T: " + path.getClosestT());

        boolean driveFinished = (SwervePosition.getPosition().sub(path.getLastPos()).mag() < Tuning.CURVE_DEADBAND);
        // boolean rotFinished = (Math.abs(currentRot - targetRot) < 0.05);
        // SwerveManager.rotateAndDrive(rotFinished ? 0.0 : rotOutput, driveFinished ? new Vector2() : driveVector);

        System.out.println("current pos: " + SwervePosition.getPosition() + " final pos: " + path.getLastPos() + " drive vector: " + driveVector.toString());



        boolean rotFinished = true;
        OperationDesertStorm.rotateAndDrive(0.0, driveFinished ? new Vector2() : new Vector2(driveVector.y, -driveVector.x));

        isFinished = driveFinished & rotFinished;

        if (isFinished) {
            System.out.println("Path Finished");
            OperationDesertStorm.rotateAndDrive(0, new Vector2());
            end(false);
        }
        OperationDesertStorm.update();
    }

    @Override
    public void end(boolean interrupted) {
        OperationDesertStorm.rotateAndDrive(0, new Vector2());
        OperationDesertStorm.update();
        if (Robot.isSimulation()) {
            OperationDesertStorm.mods[0].simModule.speed = 0;
            OperationDesertStorm.mods[1].simModule.speed = 0;
            OperationDesertStorm.mods[2].simModule.speed = 0;
            OperationDesertStorm.mods[3].simModule.speed = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
