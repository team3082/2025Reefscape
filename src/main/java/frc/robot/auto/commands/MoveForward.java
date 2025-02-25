package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePID;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.Vector2;

public class MoveForward extends Command{

    private double dist;
    public MoveForward(double dist){
        this.dist = dist;
    }

    @Override
    public void initialize() {
        SwervePID.setDestPt(SwervePosition.getPosition().add(new Vector2(0,dist).rotate(Pigeon.getRotationRad())));
    }

    @Override
    public void execute() {
        SwerveManager.rotateAndDrive(0, SwervePID.updateOutputVel());
    }

    @Override
    public boolean isFinished() {
        return SwervePID.atDest();
    }

    @Override
    public void end(boolean interrupted) {
        SwerveManager.rotateAndDrive(0, new Vector2());
    }
}
