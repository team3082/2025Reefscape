package frc.robot.openhouse;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePID;
import frc.robot.utils.Vector2;

public class TurnLeft extends Command {
  private double rotateBy;

  /**
   * A command that rotates the robot left by a specified number of degrees
   *
   * @param degrees
   */
  public TurnLeft(double degrees) {
    rotateBy = Math.toRadians(degrees);
  }

  @Override
  public void initialize() {
    SwervePID.setDestState(SwervePID.getDest(), Pigeon.getRotationRad() + rotateBy);
  }

  @Override
  public void execute() {
    double rotSpeed = SwervePID.updateOutputRot();
    SwerveManager.rotateAndDrive(rotSpeed, SwerveManager.movement);
  }

  @Override
  public boolean isFinished() {
    return SwervePID.atRot();
  }

  @Override
  public void end(boolean interrupted) {
    SwerveManager.rotateAndDrive(0, new Vector2());
  }
}
