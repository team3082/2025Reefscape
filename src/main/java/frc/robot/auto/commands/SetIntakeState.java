package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.IntakeState;
import frc.robot.subsystems.ScoringManager;

public class SetIntakeState extends Command {
    private IntakeState state;

    public SetIntakeState(IntakeState state) {
        this.state = state;
    }

    @Override
    public void initialize() {
        System.out.println("SetIntakeState Initialized");
        ScoringManager.getEndEffector().setIntakeState(state);
        end(false);
    }

    @Override
    public void execute() {
        // No execution needed
        end(false);
    }

    @Override
    public boolean isFinished() {
        // Command finishes immediately after initialization
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("SetIntakeState Ended");
    }
}
