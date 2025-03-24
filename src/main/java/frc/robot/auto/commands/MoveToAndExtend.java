package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringManager;
import frc.robot.subsystems.ScoringManager.ScoringPosition;
import frc.robot.subsystems.ScoringManager.TransitoryState;

public class MoveToAndExtend extends Command {
    private ScoringPosition scoringLevel;
    private FollowCurve followCurve;

    public MoveToAndExtend(ScoringPosition scoringLevel, FollowCurve followCurve) {
        this.scoringLevel = scoringLevel;
        this.followCurve = followCurve;
    }

    @Override
    public void initialize() {
        System.out.println("MoveToAndScore Initialized");
        followCurve.initialize();
    }

    @Override
    public void execute() {
        if (followCurve.getRemainingPathLength() < 100) {
            ScoringManager.setScoringPosition(scoringLevel);
            if (ScoringManager.getElevator().getElevatorHeight() > 25) followCurve.setMaxSpeed(0.2);
        }

        followCurve.execute();

        if (isFinished()) {
            end(false);
        }
    }

    @Override
    public boolean isFinished() {
        return followCurve.isFinished() && ScoringManager.transitoryState == TransitoryState.FINISHED && ScoringManager.scoringPosition == scoringLevel;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("MoveToAndScore Ended");
        followCurve.end(interrupted);
    }
}
