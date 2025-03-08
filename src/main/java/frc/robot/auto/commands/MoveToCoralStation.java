// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.swerve.SwervePosition;
import frc.robot.auto.commands.IntakeCoral;

public class MoveToCoralStation extends SequentialCommandGroup {
  public MoveToCoralStation(boolean testing) {
    // Red IDs 1 and 2
    // Blue IDs 12 and 13
    int aprilTagIndex = SwervePosition.getPosition().y > 0 || testing ? 2 : 1;

    addCommands(
      new ParallelCommandGroup(
        new RotateAndDriveTo(
          Constants.APRIL_TAGS[aprilTagIndex].getRotationZ() - Math.PI/2,
          Constants.APRIL_TAGS[aprilTagIndex].getCenterPosition()),
        new IntakeCoral()
        )
    );
  }
  /** Creates a new MoveToProcessor. */
  public MoveToCoralStation() {
    addCommands(new MoveToCoralStation(false));
  }
}
