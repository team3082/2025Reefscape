package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringManager;
import frc.robot.subsystems.ScoringManager.ScoringPosition;
import frc.robot.subsystems.ScoringManager.TransitoryState;

public class MoveToAndStow extends Command {
    private FollowCurve followCurve;

    public MoveToAndStow(FollowCurve followCurve) {
        this.followCurve = followCurve;
    }

    @Override
    public void initialize() {
        System.out.println("MoveToAndStow Initialized");
        followCurve.initialize();
    }

    @Override
    public void execute() {
        if (!(ScoringManager.transitoryState == TransitoryState.FINISHED && ScoringManager.scoringPosition == ScoringPosition.STOW)) {
            ScoringManager.setScoringPosition(ScoringPosition.STOW);
            followCurve.setMaxSpeed(0.5);
        } else {
            followCurve.setMaxSpeed(1.0);
        }

        followCurve.execute();

        if (isFinished()) {
            end(false);
        }
    }

    @Override
    public boolean isFinished() {
        return followCurve.isFinished() && ScoringManager.transitoryState == TransitoryState.FINISHED && ScoringManager.scoringPosition == ScoringPosition.STOW;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("MoveToAndScore Ended");
        followCurve.end(interrupted);
    }
}
