package frc.robot.openhouse;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePID;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.Vector2;

public class MoveForward extends Command {
  double moveBy;

  /**
   * A simple command that moves robot forward by a specified length
   *
   * @param inches
   */
  public MoveForward(double inches) {
    moveBy = inches;
  }

  @Override
  public void initialize() {
    // Calculate the point 'moveBy' inches away based on the rotation of the robot
    Vector2 desiredPosition =
        new Vector2(Math.cos(SwervePID.getTargetRot()), Math.sin(SwervePID.getTargetRot()))
            .norm()
            .mul(moveBy);

    SwervePID.setDestState(SwervePosition.getPosition().add(desiredPosition), SwervePID.getTargetRot());
  }

  @Override
  public void execute() {
    Vector2 vector = SwervePID.updateOutputVel();
    SwerveManager.rotateAndDrive(
        0, new Vector2(vector.x, vector.y));
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
