// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveManager;
import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;

public class SimulateCrash extends Command {
  private double duration;
  private double startTime;
  private double speed; 

  Vector2 pushVector;
  double rotation;
  
  public SimulateCrash(double duration) {
    this.duration = duration;
  }

  @Override
  public void initialize() {
    pushVector = new Vector2(Math.random(), Math.random()).norm();
    startTime = RTime.now();
  }

  @Override
  public void execute() {
    SwerveManager.rotateAndDrive(.5, pushVector);
  }

  @Override
  public void end(boolean interrupted) {
    SwerveManager.rotateAndDrive(0, new Vector2());
  }

  @Override
  public boolean isFinished() {
    return duration + startTime < RTime.now();
  }
}
