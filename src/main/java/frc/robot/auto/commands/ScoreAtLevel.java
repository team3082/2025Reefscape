package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ScoringManager.ScoringPosition;

public class ScoreAtLevel extends SequentialCommandGroup {
    public ScoreAtLevel(ScoringPosition level) { 
        addCommands(
            new MoveToScorePos(level),
            new DropCoral(1),
            new MoveToScorePos(ScoringPosition.STOW)
            );
    }
}
