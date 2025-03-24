package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePID;
import frc.robot.utils.Vector2;


public class RotateAndDriveTo extends Command{

    private Vector2 pos;
    private double rot;

    /**
     * Move and rotate to specified point
     * @param angle
     * @param position
     */
    public RotateAndDriveTo(double angle, Vector2 position){
        rot = angle;
        pos = position;
    }

    @Override
    public void initialize() {
        SwervePID.setDestState(pos, rot);
    }

    @Override
    public void execute() {
        SwerveManager.rotateAndDrive(SwervePID.updateOutputRot(), SwervePID.updateOutputVel());
    }

    @Override
    public boolean isFinished() {
        return SwervePID.atDest() && SwervePID.atRot();
    }

    @Override
    public void end(boolean interrupted) {
        SwerveManager.rotateAndDrive(0, new Vector2());
    }
}
