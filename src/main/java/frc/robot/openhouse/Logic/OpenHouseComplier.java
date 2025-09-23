package frc.robot.auto;

import java.lang.reflect.Method;
import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.openhouse.OpenHouse;
import frc.robot.openhouse.robot;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.Vector2;

public final class OpenHouseComplier{
    private static HashMap<String, AutoRoutine> allRoutines = new HashMap<String, AutoRoutine>();
    public static SendableChooser<String> autoChooser = new SendableChooser<>();

    public static class AutoRoutine {
        private String sendableName;
        private Command[] commands;
        
        public AutoRoutine(String sendableName){
            this.sendableName=sendableName;
            this.commands = robot.GetCommands();
        }

        public AutoRoutine(){
            this.sendableName="No Auto";
        }
  
        public SequentialCommandGroup getCommands(){
            if(this.sendableName=="No Auto") return new SequentialCommandGroup();
            SwervePosition.setPosition(new Vector2());
            return new SequentialCommandGroup(this.commands);
        }
    }

    public static void addRoutine(String sendableName){
        for(String name : allRoutines.keySet()){
            boolean isDuplicateName = name.equals(sendableName);
            if(isDuplicateName){
                throw new RuntimeException(sendableName+" is already a routine");
            }   
        }

        autoChooser.addOption(sendableName, sendableName);
        allRoutines.put(sendableName, new AutoRoutine(sendableName));
    }

    public static HashMap<String, AutoRoutine> getHash(){
        allRoutines.put("No Auto", new AutoRoutine());
        autoChooser.addOption("No Auto", "No Auto");
        OpenHouse classOn = new OpenHouse();
        for(Method method : classOn.getClass().getDeclaredMethods()){
            robot.init();
            try {
                method.invoke(classOn);
            } catch (Exception e) {
                // Cry, just cry
            }
            
            
            addRoutine(method.getName());
        }

        return allRoutines;
    }
}