package frc.robot.auto.routineManager;

import java.lang.reflect.Method;
import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * The RoutineManager class is responsible for managing autonomous routines.
 * It allows routines to be registered dynamically via annotated methods,
 * and provides a way to select and execute these routines through the SmartDashboard.
 */
public class RoutineManager {
    /**A HashMap to store all registered autonomous routines, with their names as keys. */ 
    private HashMap<String, CommandRoutine> routineMap = new HashMap<String, CommandRoutine>();

    /**A SendableChooser for the selection of autonomous routines via the SmartDashboard.*/ 
    public SendableChooser<String> autoSelector = new SendableChooser<String>();

    /**
     * Constructor for RoutineManager
     */
    public RoutineManager(){
        routineMap.put("No Auto", new CommandRoutine());
        autoSelector.setDefaultOption("No Auto", "No Auto");
    }
    
    /**
     * Adds a new autonomous routine to the routineMap and the autoSelector.
     *
     * @param sendableName The name of the routine as it will appear in the SmartDashboard.
     * @param initMethod A CommandSupplier that provides the command sequence for this routine.
     * @throws RuntimeException If a routine with the same name already exists.
     */
    public void addRoutine(String sendableName, CommandSupplier initMethod) {
        // Check for duplicate routine names.
        for (String name : routineMap.keySet()) {
            if (name.equals(sendableName)) {
                throw new RuntimeException(sendableName + " is already a routine");
            }
        }

        // Add the new routine to the map and the selector.
        routineMap.put(sendableName, new CommandRoutine(sendableName, initMethod));
        autoSelector.addOption(sendableName, sendableName);
    }

    /**
     * Scans a class for methods annotated with @AutoRoutine and adds them as routines.
     *
     * @param <AutoRoutineClass> The type of the class being scanned.
     * @param commandClass The class object to scan for annotated methods.
     */
    public <AutoRoutineClass> void addClass(AutoRoutineClass commandClass) {
        // Iterate over all methods in the class.
        for (Method method : commandClass.getClass().getDeclaredMethods()) {
            // Check if the method has the @AutoRoutine annotation.
            if (method.isAnnotationPresent(AutoRoutine.class)) {
                // Add the method as a routine.
                addRoutine(method.getName(), () -> {
                    try {
                        // Invoke the method to get the command sequence.
                        Object result = method.invoke(commandClass);

                        // Ensure the method returns a SequentialCommandGroup.
                        if (result instanceof SequentialCommandGroup) {
                            return (SequentialCommandGroup) result;
                        } else {
                            throw new RuntimeException(method.getName() + " doesn't return a SequentialCommandGroup");
                        }
                    } catch (Exception e) {
                        throw new RuntimeException("Error invoking method", e);
                    }
                });
            }
        }
    }
    
    /**
     * Gets the current command by calling on the routine method, this will run code in that method.
     * @return SequentialCommandGroup of the current routine
     */
    public SequentialCommandGroup getCurrentCommand(){
        return routineMap.get(autoSelector.getSelected()).getCommand();
    }

    /**
     * Gets the auto selector for choosing the routine
     * @return SendableChooser<String>
     */
    public SendableChooser<String> getAutoSelector(){
        return autoSelector;
    }
}