package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.IntakeState;
import frc.robot.Robot;
import frc.robot.subsystems.ScoringManager;
import frc.robot.utils.RTime;

public class IntakeCoral extends Command{
    private double startTime;
    

  
    @Override
    public void initialize(){
        if(Robot.isSimulation())
            startTime = RTime.now();
        ScoringManager.getEndEffector().setIntakeState(IntakeState.INTAKE_CORAL); // Intakes the piece
    }


    @Override
    public void end(boolean interrupted) {
        ScoringManager.getEndEffector().setIntakeState(IntakeState.HOLD_CORAL); // Starts holding once command ends
    }

    @Override
    public boolean isFinished() {
        return ScoringManager.getEndEffector().isHoldingCoral() || (Robot.isSimulation() && (RTime.now() - startTime) > 2);
    }

} 
