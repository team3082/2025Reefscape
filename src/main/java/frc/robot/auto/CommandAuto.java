package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CommandAuto {
    public static void init(Command command) {
       
        new SequentialCommandGroup(command).schedule();
        
      
    }
    public static void update(){
      CommandScheduler.getInstance().run();  
    }

    // TODO Uncomment when swerve is ported
    /*public static Command stop() {
        return new InstantCommand(()-> SwerveManager.rotateAndDrive(0, new Vector2()));
    }*/
} 
