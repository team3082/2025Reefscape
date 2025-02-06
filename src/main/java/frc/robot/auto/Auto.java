package frc.robot.auto;

import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.commands.FollowRobotPath;
import frc.robot.auto.commands.MoveToScorePos;
// import frc.robot.auto.commands.MoveToScorePos;
// import frc.robot.auto.commands.ScoreCoral;
import frc.robot.auto.routineManager.AutoRoutine;
import frc.robot.auto.routineManager.RoutineManager;
import frc.robot.subsystems.ScoringManager.ScoringPosition;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.Vector2;
import frc.robot.utils.trajectories.ChickenParser;
import frc.robot.utils.trajectories.RobotPath;

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

    @AutoRoutine
    public static SequentialCommandGroup pathFollowingTest() {
        System.out.println("Path Following Test");

        ChickenParser parser = new ChickenParser("src/main/deploy/ChickenPlannerSuperTest.json");
        List<RobotPath> paths = parser.getPaths();
        Vector2 startingPos = paths.get(0).getStartPos();
        SwervePosition.setPosition(startingPos);

        return new SequentialCommandGroup (
            new FollowRobotPath(paths.get(0)),
            new MoveToScorePos(ScoringPosition.L4),
            new MoveToScorePos(ScoringPosition.STOW),
            new FollowRobotPath(paths.get(1)),
            new MoveToScorePos(ScoringPosition.L3),
            new MoveToScorePos(ScoringPosition.STOW),
            new FollowRobotPath(paths.get(2)),
            new MoveToScorePos(ScoringPosition.L2),
            new MoveToScorePos(ScoringPosition.STOW),
            new FollowRobotPath(paths.get(3))
            
        );
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
            // new ScoreCoral(ScoringPosition.L4),
            // new WaitCommand(1.0),
            // new WaitCommand(1.0),
            // new ScoreCoral(ScoringPosition.L3)

            
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
    public static void autoInit() {
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
