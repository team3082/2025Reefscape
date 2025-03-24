package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringManager;

public class OverrideWristPos extends Command {
    private double targetPos;

    public OverrideWristPos(double targetPos) {
        this.targetPos = targetPos;
    }

    @Override
    public void initialize() {
        System.out.println("OverrideWristPos Initialized");
        ScoringManager.getEndEffector().setPivotAngle(targetPos);
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
        System.out.println("OverrideWristPos Ended");
    }
}
