package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.RTime;
import frc.robot.Robot;
import frc.robot.subsystems.ScoringManager;
import frc.robot.subsystems.EndEffector.IntakeState;
 
public class DropCoral extends Command  {
    private double startTime;
  
    @Override
    public void initialize(){
        // Start timer
        startTime = RTime.now();
        ScoringManager.getEndEffector().setIntakeState(IntakeState.DROP_CORAL); // Start dropping

    }


    @Override
    public void end(boolean interrupted) {
        ScoringManager.getEndEffector().setIntakeState(IntakeState.HOLD_CORAL); // Stop dropping
    }

    @Override
    public boolean isFinished() {
        // Check if specified time has elapsed
        return (Robot.isSimulation() && RTime.now() - startTime > 1) || !ScoringManager.getEndEffector().holdingPiece;
    }
}
