package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.commands.MoveToScorePos;
import frc.robot.Constants;
import frc.robot.auto.commands.ClimbToPos;
import frc.robot.auto.commands.DropCoral;
import frc.robot.auto.commands.IntakeCoral;
import frc.robot.auto.routineManager.AutoRoutine;
import frc.robot.auto.routineManager.RoutineManager;
import frc.robot.subsystems.ScoringManager.ScoringPosition;
import frc.robot.auto.commands.RotateAndDriveTo;
import frc.robot.subsystems.Climber.ClimbState;


/**
 * Manages autonomous routines for the robot.
 * Uses {@link RoutineManager} to automatically detect and handle routines
 * annotated with {@link AutoRoutine}.
 */
public class Auto {
    public static RoutineManager routineManager;

    /**
     * Gets the auto selector from {@link RoutineManager}
     * @return SendableChooser<String>
     */
    public static SendableChooser<String> getAutoSelector(){
        return routineManager.getAutoSelector();
    }


    /**
     * Example autonomous routine #1.
     * This method will be automatically detected and registered by {@link RoutineManager}.
     */
    @AutoRoutine
    public static SequentialCommandGroup autoRoutineOne() {
        // Define autonomous routine logic here.
        return new SequentialCommandGroup(
            Commands.runOnce(()->System.out.println("Test One")),
            Commands.runOnce(()->System.out.println("Test Two")),
            Commands.runOnce(()->System.out.println("Test Three"))
        );
    }

    /**
     * Example autonomous routine #2.
     * This method will also be automatically detected and registered.
     */
    @AutoRoutine
    public static SequentialCommandGroup autoRoutineTwo() {
        // Define autonomous routine logic here.
        return new SequentialCommandGroup(
            Commands.runOnce(()->System.out.println("But")),
            new WaitCommand(1.0),
            Commands.runOnce(()->System.out.println("When")),
            new WaitCommand(1.0),
            Commands.runOnce(()->System.out.println("You Close Your Eyes"))
        );
    }

    @AutoRoutine
    public SequentialCommandGroup scoringManagerTest(){
        return new SequentialCommandGroup(
            new RotateAndDriveTo(Constants.APRIL_TAGS[10].getRotationZ() + Math.PI/2,Constants.APRIL_TAGS[10].getLeftPosition()),
            new MoveToScorePos(ScoringPosition.L4),
            new DropCoral(1),
            new WaitCommand(1.0),
            new MoveToScorePos(ScoringPosition.STOW),
            new RotateAndDriveTo(Constants.APRIL_TAGS[2].getRotationZ() - Math.PI/2,Constants.APRIL_TAGS[2].getLeftPosition()),
            new IntakeCoral(),
            new RotateAndDriveTo(Constants.APRIL_TAGS[8].getRotationZ() + Math.PI/2,Constants.APRIL_TAGS[8].getLeftPosition()),
            new MoveToScorePos(ScoringPosition.L4),
            new DropCoral(1),
            new MoveToScorePos(ScoringPosition.STOW)
        );
    }

    @AutoRoutine
    public SequentialCommandGroup climbTest(){
        return new SequentialCommandGroup(
            new WaitCommand(3.0),
            new ClimbToPos(ClimbState.CLIMBING),
            new WaitCommand(3.0),
            new ClimbToPos(ClimbState.RESTING)
        );
    }


    /**
     * Initializes the autonomous system by creating a {@link RoutineManager}
     * instance and registering all routines in this class.
     */
    public static void init() {
        routineManager = new RoutineManager();
        routineManager.addClass(new Auto());
    }

    /**
     * Schedules the currently selected autonomous command.
     * Should be called at the start of autonomous mode.
     */
    public static void startRoutine() {
        CommandScheduler.getInstance().enable();
        routineManager.getCurrentCommand().schedule();
    }

    /**
     * Runs the {@link CommandScheduler}, ensuring commands are executed.
     * Should be called periodically during autonomous mode.
     */
    public static void update() {
        CommandScheduler.getInstance().run();
    }

    /**
     * Disables the {@link CommandScheduler}, ensuring commands are turned off.
     * Should be called during robot disabled
     */
    public static void disable() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().disable();
    }
}
