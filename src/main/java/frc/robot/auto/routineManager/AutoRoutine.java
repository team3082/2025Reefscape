package frc.robot.auto.routineManager;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.RUNTIME)
@Target({ElementType.METHOD, ElementType.TYPE})
/** The AutoRoutine annotation registers a method to be dynamically added to the list of auto routines.
 * The method should return the type of SequentialCommandGroup or a runtime error will be thrown.
 * Futhermore, remember that this method will be called on the starting of an auto routine.
 */
public @interface AutoRoutine {}

