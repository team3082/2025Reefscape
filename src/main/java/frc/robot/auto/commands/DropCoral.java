package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.RTime;
import frc.robot.subsystems.LebronJames;
import frc.robot.subsystems.ArmedForces.IntakeState;
 
public class DropCoral extends Command  {
    private double waitSeconds;
    private double startTime;

    /**
     * A command that sets the end effector to the DROP_PIECE state for a specified time
     * @param seconds
     */
    public DropCoral(double seconds){
        this.waitSeconds = seconds;
    }
  
    @Override
    public void initialize(){
        // Start timer
        startTime = RTime.now();
        LebronJames.getEndEffector().setIntakeState(IntakeState.DROP_PIECE); // Start dropping

    }


    @Override
    public void end(boolean interrupted) {
        LebronJames.getEndEffector().setIntakeState(IntakeState.HOLD_PIECE); // Stop dropping
    }

    @Override
    public boolean isFinished() {
        // Check if specified time has elapsed
        if(RTime.now() - startTime > waitSeconds) {
            return true;
        } 
        return false;
    }
}
