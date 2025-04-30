package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringManager;
import frc.robot.subsystems.ScoringManager.ScoringPosition;
import frc.robot.subsystems.ScoringManager.TransitoryState;

public class MoveToAndStowTrapezoidal extends Command {
    private FollowCurveTrapezoidal followCurve;

    public MoveToAndStowTrapezoidal(FollowCurveTrapezoidal followCurve) {
        this.followCurve = followCurve;
    }

    @Override
    public void initialize() {
        System.out.println("MoveToAndStow Initialized");
        followCurve.initialize();
    }

    @Override
    public void execute() {
        ScoringManager.setScoringPosition(ScoringPosition.STOW);
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
