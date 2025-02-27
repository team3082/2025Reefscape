// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveToStation extends SequentialCommandGroup {
  /** Creates a new MoveToStation. */
  public MoveToStation(boolean isBlueSide, boolean isLeftStation) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
      int aprilTagIndex = isBlueSide ? (isLeftStation ? 13 : 12) : (isLeftStation ? 1 : 2);
      addCommands(new RotateAndDriveTo(Constants.APRIL_TAGS[aprilTagIndex].getRotationZ() + Math.PI/2,
                          Constants.APRIL_TAGS[aprilTagIndex].getCenterPosition()));
    
  }
}
