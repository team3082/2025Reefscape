package frc.robot.auto.routineManager;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

@FunctionalInterface
/**Functional Interface for a method that returns a SequentialCommandGroup */
public interface CommandSupplier {
    SequentialCommandGroup getCommands();
}

