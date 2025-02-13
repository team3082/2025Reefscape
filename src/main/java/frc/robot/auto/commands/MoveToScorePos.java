// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TheGreatWall;
import frc.robot.subsystems.LebronJames;
import frc.robot.subsystems.LebronJames.ScoringPosition;
import frc.robot.subsystems.LebronJames.TransitoryState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToScorePos extends Command {
  private ScoringPosition pos;

  /** Creates a new MoveToScorePos. */
  public MoveToScorePos(ScoringPosition pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pos = pos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LebronJames.setScoringLevel(pos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LebronJames.update();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (LebronJames.transitoryState == TransitoryState.FINISHED){
      return true;
    } else {
      return false;
    }
  }
}
