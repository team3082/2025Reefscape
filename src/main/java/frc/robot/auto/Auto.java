package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.commands.MoveToScorePos;
import frc.robot.auto.commands.MoveToStation;
import frc.robot.Constants;
import frc.robot.auto.commands.CrashTest;
import frc.robot.auto.commands.DropCoral;
import frc.robot.auto.commands.FollowChickenRoutine;
import frc.robot.auto.commands.IntakeCoral;
import frc.robot.auto.routineManager.AutoRoutine;
import frc.robot.auto.routineManager.RoutineManager;
import frc.robot.subsystems.ScoringManager.ScoringPosition;
import frc.robot.auto.commands.RotateAndDriveTo;
import frc.robot.auto.commands.SimulateCrash;


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

    public SequentialCommandGroup scoreAt(int aprilTag, ScoringPosition position){
        return new SequentialCommandGroup(
            new RotateAndDriveTo(Constants.APRIL_TAGS[aprilTag].getRotationZ() + Math.PI/2,Constants.APRIL_TAGS[aprilTag].getLeftPosition()),
            new MoveToScorePos(ScoringPosition.L4),
            new DropCoral(1),
            new WaitCommand(1.0),
            new MoveToScorePos(ScoringPosition.STOW)
        );
    }

    public SequentialCommandGroup intakeAt(int aprilTag){
        return new SequentialCommandGroup(
            new MoveToScorePos(ScoringPosition.STOW),
            new RotateAndDriveTo(Constants.APRIL_TAGS[aprilTag].getRotationZ() - Math.PI/2,Constants.APRIL_TAGS[aprilTag].getLeftPosition()),
            new WaitCommand(2)
        );
    }


    @AutoRoutine
    public SequentialCommandGroup TushPush2Piece(){
        return new FollowChickenRoutine("TushPush2Piece", true,
            scoreAt(9, ScoringPosition.L4), 
            intakeAt(2), 
            scoreAt(8, ScoringPosition.L4)
        );
    }

    @AutoRoutine
    public SequentialCommandGroup ThreePiece(){
        return new FollowChickenRoutine("3Piece", true,
            scoreAt(9, ScoringPosition.L4),
            intakeAt(2), 
            scoreAt(8, ScoringPosition.L4),
            intakeAt(2), 
            scoreAt(8, ScoringPosition.L4)
        );
    }


    @AutoRoutine
    public SequentialCommandGroup ExtreemTestPath(){
        return new FollowChickenRoutine("ExtreemTestPath", true);
    }

    @AutoRoutine
    public SequentialCommandGroup BowlingCrash(){
        return new FollowChickenRoutine("bowlngCrash", true);
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
