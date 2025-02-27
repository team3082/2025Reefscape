// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class MoveToProcessor extends SequentialCommandGroup {
  /** Creates a new MoveToProcessor. */
  public MoveToProcessor(boolean isBlueSide) {
    int aprilTagIndex = isBlueSide ? 3 : 16;
    addCommands(new RotateAndDriveTo(Constants.APRIL_TAGS[aprilTagIndex].getRotationZ() + Math.PI/2,
                        Constants.APRIL_TAGS[aprilTagIndex].getCenterPosition()));
  }
}
