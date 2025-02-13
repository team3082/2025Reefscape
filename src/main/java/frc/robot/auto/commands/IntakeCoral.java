package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmedForces.IntakeState;
import frc.robot.subsystems.LebronJames;
import frc.robot.utils.RTime;

public class IntakeCoral extends Command{
    private double waitSeconds;
    private double startTime;
    
    /**
     * A command that sets the end effector to the INTAKE_PIECE state for a specified time
     * @param seconds
     */
    public IntakeCoral(double seconds){
        this.waitSeconds = seconds;
    }
  
    @Override
    public void initialize(){
        // Start timer
        startTime = RTime.now();
        LebronJames.getEndEffector().setIntakeState(IntakeState.INTAKE_PIECE); // Intakes the piece

    }


    @Override
    public void end(boolean interrupted) {
        LebronJames.getEndEffector().setIntakeState(IntakeState.HOLD_PIECE); // Starts holding once command ends
    }

    @Override
    public boolean isFinished() {
        // Check if specified time has elapsed, if so finish
        if(RTime.now() - startTime > waitSeconds) {
            return true;
        } 
        return false;
    }

} 
