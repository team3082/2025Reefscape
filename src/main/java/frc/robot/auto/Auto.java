package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Tuning;
import frc.robot.auto.commands.AlignToReef;
import frc.robot.auto.commands.FollowCurve;
import frc.robot.auto.commands.IntakeCoral;
import frc.robot.auto.commands.MoveToCoralStation;
import frc.robot.auto.commands.MoveToScorePos;
import frc.robot.auto.commands.OverrideWristPos;
import frc.robot.auto.routineManager.AutoRoutine;
import frc.robot.auto.routineManager.RoutineManager;
import frc.robot.subsystems.EndEffector.IntakeState;
import frc.robot.subsystems.ScoringManager.ScoringPosition;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.trajectories.LinearBezier;
import frc.robot.auto.commands.ScoreAtLevel;
import frc.robot.auto.commands.SetIntakeState;

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

    /** scores G4 - removes back algae */
    @AutoRoutine
    public SequentialCommandGroup onePieceMiddleRight() {
        SwervePosition.setPosition(Constants.MIDDLE_STARTING_POS);
        Pigeon.setYawRad((Constants.APRIL_TAGS[10].getRotationZ() + Math.PI / 2.0) % (2.0 * Math.PI));
        return new SequentialCommandGroup (
            new FollowCurve(Tuning.AutoPaths.START_TO_G, Constants.APRIL_TAGS[10].getRotationZ(), 0.5, 0.2),
            new ScoreAtLevel(ScoringPosition.L4),
            new FollowCurve(Tuning.AutoPaths.G_TO_WAIT, Constants.APRIL_TAGS[10].getRotationZ(), 0.5, 0.2),
            new MoveToScorePos(ScoringPosition.ALGAE1),
            new SetIntakeState(IntakeState.DROP_CORAL),
            new FollowCurve(Tuning.AutoPaths.WAIT_TO_BACK_ALGAE, Constants.APRIL_TAGS[10].getRotationZ(), 0.2, 0.2),
            new FollowCurve(Tuning.AutoPaths.BACK_ALGAE_TO_WAIT, Constants.APRIL_TAGS[10].getRotationZ(), 0.2, 0.2),
            new OverrideWristPos((5.0 * Math.PI) / 8.0),
            new SetIntakeState(IntakeState.INTAKE_ALGAE),
            new WaitCommand(1),
            new SetIntakeState(IntakeState.HOLD_CORAL),
            new MoveToScorePos(ScoringPosition.STOW)
        );
    }

    /** scores G4 - removes back algae */
    @AutoRoutine
    public SequentialCommandGroup onePieceMiddleLeft() {
        SwervePosition.setPosition(Constants.MIDDLE_STARTING_POS);
        Pigeon.setYawRad((Constants.APRIL_TAGS[10].getRotationZ() + Math.PI / 2.0) % (2.0 * Math.PI));
        return new SequentialCommandGroup (
            new FollowCurve(Tuning.AutoPaths.START_TO_H, Constants.APRIL_TAGS[10].getRotationZ(), 0.5, 0.2),
            new ScoreAtLevel(ScoringPosition.L4),
            new FollowCurve(Tuning.AutoPaths.H_TO_WAIT, Constants.APRIL_TAGS[10].getRotationZ(), 0.5, 0.2),
            new MoveToScorePos(ScoringPosition.ALGAE1),
            new SetIntakeState(IntakeState.DROP_CORAL),
            new FollowCurve(Tuning.AutoPaths.WAIT_TO_BACK_ALGAE, Constants.APRIL_TAGS[10].getRotationZ(), 0.2, 0.2),
            new FollowCurve(Tuning.AutoPaths.BACK_ALGAE_TO_WAIT, Constants.APRIL_TAGS[10].getRotationZ(), 0.2, 0.2),
            new OverrideWristPos((5.0 * Math.PI) / 8.0),
            new SetIntakeState(IntakeState.INTAKE_ALGAE),
            new WaitCommand(1),
            new SetIntakeState(IntakeState.HOLD_CORAL),
            new MoveToScorePos(ScoringPosition.STOW)
        );
    }

    /** scores E4 - intakes */
    @AutoRoutine
    public SequentialCommandGroup onePieceRight() {
        SwervePosition.setPosition(Constants.RIGHT_STARTING_POS);
        Pigeon.setYawRad((Constants.APRIL_TAGS[9].getRotationZ() + Math.PI / 2.0) % (2.0 * Math.PI));
        return new SequentialCommandGroup(
            new FollowCurve(Tuning.AutoPaths.START_TO_E, Constants.APRIL_TAGS[9].getRotationZ(), 0.5, 0.3),
            new ScoreAtLevel(ScoringPosition.L4)
        );
    }

    /** scores J4 - intakes */
    @AutoRoutine
    public SequentialCommandGroup onePieceLeft() {
        SwervePosition.setPosition(Constants.LEFT_STARTING_POS);
        Pigeon.setYawRad((Constants.APRIL_TAGS[11].getRotationZ() + Math.PI / 2.0) % (2.0 * Math.PI));
        return new SequentialCommandGroup(
            new FollowCurve(Tuning.AutoPaths.START_TO_J, Constants.APRIL_TAGS[11].getRotationZ(), 0.5, 0.3),
            new ScoreAtLevel(ScoringPosition.L4)
        );
    }

    /** scores E4 - intakes */
    @AutoRoutine
    public SequentialCommandGroup oneHalfPieceRight() {
        SwervePosition.setPosition(Constants.RIGHT_STARTING_POS);
        Pigeon.setYawRad((Constants.APRIL_TAGS[9].getRotationZ() + Math.PI / 2.0) % (2.0 * Math.PI));
        return new SequentialCommandGroup(
            new FollowCurve(Tuning.AutoPaths.START_TO_E, Constants.APRIL_TAGS[9].getRotationZ(), 0.5, 0.3),
            new ScoreAtLevel(ScoringPosition.L4),
            new FollowCurve(Tuning.AutoPaths.E_TO_STATION, Constants.APRIL_TAGS[2].getRotationZ() + Math.PI, 1.0, 0.3),
            new IntakeCoral()
        );
    }

    /** scores J4 - intakes */
    @AutoRoutine
    public SequentialCommandGroup oneHalfPieceLeft() {
        SwervePosition.setPosition(Constants.LEFT_STARTING_POS);
        Pigeon.setYawRad((Constants.APRIL_TAGS[11].getRotationZ() + Math.PI / 2.0) % (2.0 * Math.PI));
        return new SequentialCommandGroup(
            new FollowCurve(Tuning.AutoPaths.START_TO_J, Constants.APRIL_TAGS[11].getRotationZ(), 0.5, 0.3),
            new ScoreAtLevel(ScoringPosition.L4),
            new FollowCurve(Tuning.AutoPaths.J_TO_STATION, Constants.APRIL_TAGS[1].getRotationZ() + Math.PI, 1.0, 0.3),
            new IntakeCoral()
        );
    }

    /** scores E4 - intakes - scores D4 */
    @AutoRoutine
    public SequentialCommandGroup twoPieceRight() {
        SwervePosition.setPosition(Constants.RIGHT_STARTING_POS);
        Pigeon.setYawRad((Constants.APRIL_TAGS[9].getRotationZ() + Math.PI / 2.0) % (2.0 * Math.PI));
        return new SequentialCommandGroup(
            new FollowCurve(Tuning.AutoPaths.START_TO_E, Constants.APRIL_TAGS[9].getRotationZ(), 0.5, 0.3),
            new ScoreAtLevel(ScoringPosition.L4),
            new FollowCurve(Tuning.AutoPaths.E_TO_STATION, Constants.APRIL_TAGS[2].getRotationZ() + Math.PI, 1.0, 0.3),
            new IntakeCoral(),
            new FollowCurve(Tuning.AutoPaths.STATION_TO_D, Constants.APRIL_TAGS[8].getRotationZ(), 0.5, 0.3),
            new ScoreAtLevel(ScoringPosition.L4)
        );
    }

    /** scores J4 - intakes - scores K4 */
    @AutoRoutine
    public SequentialCommandGroup twoPieceLeft() {
        SwervePosition.setPosition(Constants.LEFT_STARTING_POS);
        Pigeon.setYawRad((Constants.APRIL_TAGS[11].getRotationZ() + Math.PI / 2.0) % (2.0 * Math.PI));
        return new SequentialCommandGroup(
            new FollowCurve(Tuning.AutoPaths.START_TO_J, Constants.APRIL_TAGS[11].getRotationZ(), 0.5, 0.3),
            new ScoreAtLevel(ScoringPosition.L4),
            new FollowCurve(Tuning.AutoPaths.J_TO_STATION, Constants.APRIL_TAGS[1].getRotationZ() + Math.PI, 1.0, 0.3),
            new IntakeCoral(),
            new FollowCurve(Tuning.AutoPaths.STATION_TO_K, Constants.APRIL_TAGS[6].getRotationZ(), 0.5, 0.3),
            new ScoreAtLevel(ScoringPosition.L4)
        );
    }

    /** scores E4 - intakes - scores D4 - intakes */
    @AutoRoutine
    public SequentialCommandGroup twoHalfPieceRight() {
        SwervePosition.setPosition(Constants.RIGHT_STARTING_POS);
        Pigeon.setYawRad((Constants.APRIL_TAGS[9].getRotationZ() + Math.PI / 2.0) % (2.0 * Math.PI));
        return new SequentialCommandGroup(
            new FollowCurve(Tuning.AutoPaths.START_TO_E, Constants.APRIL_TAGS[9].getRotationZ(), 0.5, 0.3),
            new ScoreAtLevel(ScoringPosition.L4),
            new FollowCurve(Tuning.AutoPaths.E_TO_STATION, Constants.APRIL_TAGS[2].getRotationZ() + Math.PI, 1.0, 0.3),
            new IntakeCoral(),
            new FollowCurve(Tuning.AutoPaths.STATION_TO_D, Constants.APRIL_TAGS[8].getRotationZ(), 0.5, 0.3),
            new ScoreAtLevel(ScoringPosition.L4),
            new FollowCurve(Tuning.AutoPaths.D_TO_STATION, Constants.APRIL_TAGS[2].getRotationZ() + Math.PI, 1.0, 0.3),
            new IntakeCoral()
        );
    }

    /** scores J4 - intakes - scores K4 - intakes */
    @AutoRoutine
    public SequentialCommandGroup twoHalfPieceLeft() {
        SwervePosition.setPosition(Constants.LEFT_STARTING_POS);
        Pigeon.setYawRad((Constants.APRIL_TAGS[11].getRotationZ() + Math.PI / 2.0) % (2.0 * Math.PI));
        return new SequentialCommandGroup(
            new FollowCurve(Tuning.AutoPaths.START_TO_J, Constants.APRIL_TAGS[11].getRotationZ(), 0.5, 0.3),
            new ScoreAtLevel(ScoringPosition.L4),
            new FollowCurve(Tuning.AutoPaths.J_TO_STATION, Constants.APRIL_TAGS[1].getRotationZ() + Math.PI, 1.0, 0.3),
            new IntakeCoral(),
            new FollowCurve(Tuning.AutoPaths.STATION_TO_K, Constants.APRIL_TAGS[6].getRotationZ(), 0.5, 0.3),
            new ScoreAtLevel(ScoringPosition.L4),
            new FollowCurve(Tuning.AutoPaths.K_TO_STATION, Constants.APRIL_TAGS[1].getRotationZ() + Math.PI, 1.0, 0.3),
            new IntakeCoral()
        );
    }

    /** scores E4 - intakes - scores D4 - intakes - scores C4 */
    @AutoRoutine
    public SequentialCommandGroup threePieceRight() {
        SwervePosition.setPosition(Constants.RIGHT_STARTING_POS);
        Pigeon.setYawRad((Constants.APRIL_TAGS[9].getRotationZ() + Math.PI / 2.0) % (2.0 * Math.PI));
        return new SequentialCommandGroup(
            new FollowCurve(Tuning.AutoPaths.START_TO_E, Constants.APRIL_TAGS[9].getRotationZ(), 0.5, 0.3),
            new ScoreAtLevel(ScoringPosition.L4),
            new FollowCurve(Tuning.AutoPaths.E_TO_STATION, Constants.APRIL_TAGS[2].getRotationZ() + Math.PI, 1.0, 0.3),
            new IntakeCoral(),
            new FollowCurve(Tuning.AutoPaths.STATION_TO_D, Constants.APRIL_TAGS[8].getRotationZ(), 0.5, 0.3),
            new ScoreAtLevel(ScoringPosition.L4),
            new FollowCurve(Tuning.AutoPaths.D_TO_STATION, Constants.APRIL_TAGS[2].getRotationZ() + Math.PI, 1.0, 0.3),
            new IntakeCoral(),
            new FollowCurve(Tuning.AutoPaths.STATION_TO_C, Constants.APRIL_TAGS[8].getRotationZ(), 0.5, 0.3),
            new ScoreAtLevel(ScoringPosition.L4)
        );
    }

    /** scores J4 - intakes - scores K4 - intakes - scores L4 */
    @AutoRoutine
    public SequentialCommandGroup threePieceLeft() {
        SwervePosition.setPosition(Constants.LEFT_STARTING_POS);
        Pigeon.setYawRad((Constants.APRIL_TAGS[11].getRotationZ() + Math.PI / 2.0) % (2.0 * Math.PI));
        return new SequentialCommandGroup(
            new FollowCurve(Tuning.AutoPaths.START_TO_J, Constants.APRIL_TAGS[11].getRotationZ(), 0.5, 0.3),
            new ScoreAtLevel(ScoringPosition.L4),
            new FollowCurve(Tuning.AutoPaths.J_TO_STATION, Constants.APRIL_TAGS[1].getRotationZ() + Math.PI, 1.0, 0.3),
            new IntakeCoral(),
            new FollowCurve(Tuning.AutoPaths.STATION_TO_K, Constants.APRIL_TAGS[6].getRotationZ(), 0.5, 0.3),
            new ScoreAtLevel(ScoringPosition.L4),
            new FollowCurve(Tuning.AutoPaths.K_TO_STATION, Constants.APRIL_TAGS[1].getRotationZ() + Math.PI, 1.0, 0.3),
            new IntakeCoral(),
            new FollowCurve(Tuning.AutoPaths.STATION_TO_L, Constants.APRIL_TAGS[6].getRotationZ(), 0.5, 0.3),
            new ScoreAtLevel(ScoringPosition.L4)
        );
    }

    @AutoRoutine
    public SequentialCommandGroup scoringManagerTest(){
        return new SequentialCommandGroup(
            new AlignToReef(7, true),
            new ScoreAtLevel(ScoringPosition.L4),
            new MoveToCoralStation(true),
            new IntakeCoral(),
            new AlignToReef(7, false),
            new ScoreAtLevel(ScoringPosition.L4) 
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
