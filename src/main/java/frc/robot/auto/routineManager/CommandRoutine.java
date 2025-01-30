package frc.robot.auto.routineManager;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Represents an autonomous routine, including its name, command sequence, and initialization method.
 */
public class CommandRoutine {
    private String sendableName; // The name of the routine as it appears in the SmartDashboard.
    private CommandSupplier initMethod; // A supplier that provides the commands for this routine.

    /**
     * Constructs an AutoRoutine with a specific name and initialization method.
     *  
     * @param sendableName The name of the routine as it will appear in the SmartDashboard.
     * @param initMethod A CommandSupplier that provides the command sequence for this routine.
     */
    public CommandRoutine(String sendableName, CommandSupplier initMethod) {
        this.sendableName = sendableName;
        this.initMethod = initMethod;
    }

    /**
     * Default constructor for creating a "No Auto" routine.
     */
    public CommandRoutine() {
        this.sendableName = "No Auto";
    }

    /**
     * Initializes the routine by fetching the commands from the initMethod.
     * If the routine is "No Auto," this method does nothing.
     * @return The SequentialCommandGroup representing the routine's commands.
     */
    public SequentialCommandGroup getCommand() {
        // If it's the "No Auto" routine, do nothing but print it out.
        if (this.sendableName.equals("No Auto")){
            return new SequentialCommandGroup(new InstantCommand(()->System.out.println("NO AUTO ROUTINE SELECTED")));
        }

        // Fetch the commands.
        return initMethod.getCommands();
    }

    /**
     * Returns the name of the routine.
     *
     * @return The name of the routine.
     */
    public String getName() {
        return sendableName;
    }
}
