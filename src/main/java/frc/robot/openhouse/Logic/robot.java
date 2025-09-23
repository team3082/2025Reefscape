package frc.robot.openhouse;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;

public class robot {
    private static ArrayList<Command> commandList;

    public static void init(){
        commandList = new ArrayList<>();
    }

    public static Command[] GetCommands(){
        Command[] array = new Command[commandList.size()];
        for(int index = 0; index<commandList.size(); index++){
            array[index] = commandList.get(index);
        }
        return array;
    }

    /**
     * Moves the robot forward by a specified number of inches
     * @param inches
     */
    public static void moveForward(double inches) {
        commandList.add(new MoveForward(inches));
    }

    /**
     * A command that rotates the robot left by a specified number of degrees
     * @param degrees
     */
    public static void turnLeft(double degrees) {
        commandList.add(new TurnLeft(degrees));
    }

    /**
     * A command that rotates the robot right by a specified number of degrees
     * @param degrees
     */
    public static void turnRight(double degrees) {
        commandList.add(new TurnRight(degrees));
    }
}
