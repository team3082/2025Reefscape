package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;

public class AlignToReef extends SequentialCommandGroup {
    public AlignToReef(int id, boolean right){
        if (Robot.getAllianceMultiplier() == -1) id += 8;
        if (right){
            addCommands(
                new RotateAndDriveTo(Constants.APRIL_TAGS[id].getRotationZ() + Math.PI/2,Constants.APRIL_TAGS[id].getRightPosition())
            );
            return;
        }

        addCommands(
            new RotateAndDriveTo(Constants.APRIL_TAGS[id].getRotationZ() + Math.PI/2,Constants.APRIL_TAGS[id].getLeftPosition())
        );
    }
}
