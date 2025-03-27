package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.auto.commands.AlignToReef;
import frc.robot.auto.commands.DropCoral;
import frc.robot.auto.commands.FollowCurve;
import frc.robot.auto.commands.IntakeCoral;
import frc.robot.auto.commands.MoveToAndExtend;
import frc.robot.auto.commands.MoveToAndStow;
import frc.robot.auto.commands.MoveToCoralStation;
import frc.robot.auto.commands.MoveToScorePos;
import frc.robot.auto.commands.OverrideWristPos;
import frc.robot.auto.routineManager.AutoRoutine;
import frc.robot.auto.routineManager.RoutineManager;
import frc.robot.subsystems.EndEffector.IntakeState;
import frc.robot.subsystems.ScoringManager.ScoringPosition;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwervePosition;
import frc.robot.vision.VisionManager;
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
    public SequentialCommandGroup onePieceMiddleRightAlgae() {
        if (Robot.isSimulation()) SwervePosition.setPosition(Constants.MIDDLE_STARTING_POS);

        Pigeon.setYawRad((3.0 * Math.PI) / 2.0);

        return new SequentialCommandGroup (
            new FollowCurve(Tuning.AutoPaths.START_TO_G, Constants.REEF_POSITIONS.G.getRotation(), 0.75, 0.2),
            new ScoreAtLevel(ScoringPosition.L4),
            new FollowCurve(Tuning.AutoPaths.G_TO_WAIT, Constants.APRIL_TAGS[10].getRotationZ(), 0.75, 0.2, 0.075),
            new MoveToScorePos(ScoringPosition.ALGAE1),
            new SetIntakeState(IntakeState.DROP_CORAL),
            new FollowCurve(Tuning.AutoPaths.WAIT_TO_BACK_ALGAE, Constants.APRIL_TAGS[10].getRotationZ(), 0.2, 0.2, 0.075),
            new WaitCommand(0.25),
            new FollowCurve(Tuning.AutoPaths.BACK_ALGAE_TO_WAIT, Constants.APRIL_TAGS[10].getRotationZ(), 0.4, 0.2, 0.075),
            new OverrideWristPos((5.0 * Math.PI) / 8.0),
            new SetIntakeState(IntakeState.INTAKE_ALGAE),
            new WaitCommand(1),
            new SetIntakeState(IntakeState.HOLD_CORAL),
            new MoveToScorePos(ScoringPosition.STOW)
        );
    }

    /** scores G4 - removes back algae */
    @AutoRoutine
    public SequentialCommandGroup onePieceMiddleLeftAlgae() {
        if( Robot.isSimulation()) SwervePosition.setPosition(Constants.MIDDLE_STARTING_POS);

        Pigeon.setYawRad((3.0 * Math.PI) / 2.0);

        return new SequentialCommandGroup (
            new FollowCurve(Tuning.AutoPaths.START_TO_H, Constants.REEF_POSITIONS.H.getRotation(), 0.75, 0.2),
            new ScoreAtLevel(ScoringPosition.L4),
            new FollowCurve(Tuning.AutoPaths.H_TO_WAIT, Constants.REEF_POSITIONS.H.getRotation(), 0.75, 0.2),
            new MoveToScorePos(ScoringPosition.ALGAE1),
            new SetIntakeState(IntakeState.DROP_CORAL),
            new FollowCurve(Tuning.AutoPaths.WAIT_TO_BACK_ALGAE, Constants.REEF_POSITIONS.H.getRotation(), 0.2, 0.2),
            new WaitCommand(0.25),
            new FollowCurve(Tuning.AutoPaths.BACK_ALGAE_TO_WAIT, Constants.REEF_POSITIONS.H.getRotation(), 0.4, 0.2),
            new OverrideWristPos((5.0 * Math.PI) / 8.0),
            new SetIntakeState(IntakeState.INTAKE_ALGAE),
            new WaitCommand(1),
            new SetIntakeState(IntakeState.HOLD_CORAL),
            new MoveToScorePos(ScoringPosition.STOW)
        );
    }


    @AutoRoutine
    public SequentialCommandGroup threeHalfPieceRight() {
        if (Robot.isSimulation()) SwervePosition.setPosition(Constants.RIGHT_STARTING_POS);

        Pigeon.setYawRad((3.0 * Math.PI) / 2.0);

        return new SequentialCommandGroup(
            new InstantCommand(() -> VisionManager.disableLeftCam()),
            new MoveToAndExtend(ScoringPosition.L4, new FollowCurve(Tuning.AutoPaths.START_TO_E, Constants.REEF_POSITIONS.E.getRotation(), 0.75, 0.3)),
            Commands.waitSeconds(.2),
            new DropCoral(),
            new MoveToAndStow(new FollowCurve(Tuning.AutoPaths.E_TO_STATION, Constants.APRIL_TAGS[2].getRotationZ() + Math.PI, 1.0, 0.3, 0.075)),
            new IntakeCoral(),
            new InstantCommand(() -> VisionManager.enableLeftCam()),

            new InstantCommand(() -> VisionManager.disableRightCam()),
            new MoveToAndExtend(ScoringPosition.L4, new FollowCurve(Tuning.AutoPaths.STATION_TO_D, Constants.REEF_POSITIONS.D.getRotation(), 0.75, 0.3)),
            Commands.waitSeconds(.2),
            new DropCoral(),
            new MoveToAndStow(new FollowCurve(Tuning.AutoPaths.D_TO_STATION, Constants.APRIL_TAGS[2].getRotationZ() + Math.PI, 1.0, 0.3, 0.075)),
            new IntakeCoral(),
            new InstantCommand(() -> VisionManager.enableRightCam()),
            
            new InstantCommand(() -> VisionManager.disableLeftCam()),
            new MoveToAndExtend(ScoringPosition.L4, new FollowCurve(Tuning.AutoPaths.STATION_TO_C, Constants.REEF_POSITIONS.C.getRotation(), 0.75, 0.3)),
            Commands.waitSeconds(.2),
            new DropCoral(),
            new MoveToAndStow(new FollowCurve(Tuning.AutoPaths.C_TO_STATION, Constants.APRIL_TAGS[2].getRotationZ() + Math.PI, 1.0, 0.3, 0.075)),
            new IntakeCoral()
        );
    }

    @AutoRoutine
    public SequentialCommandGroup threeHalfPieceLeft() {
        if (Robot.isSimulation()) {
            SwervePosition.setPosition(Constants.LEFT_STARTING_POS);
            Pigeon.setYawRad((Constants.APRIL_TAGS[11].getRotationZ() + Math.PI / 2.0) % (2.0 * Math.PI));
        }
        Pigeon.setYawRad((3.0 * Math.PI) / 2.0);
        return new SequentialCommandGroup(
            new InstantCommand(() -> VisionManager.disableRightCam()),
            new MoveToAndExtend(ScoringPosition.L4, new FollowCurve(Tuning.AutoPaths.START_TO_J, Constants.REEF_POSITIONS.J.getRotation(), 0.75, 0.3)),
            Commands.waitSeconds(.2),
            new DropCoral(),
            new MoveToAndStow(new FollowCurve(Tuning.AutoPaths.J_TO_STATION, Constants.APRIL_TAGS[1].getRotationZ() + Math.PI, 0.9, 0.3, 0.075)),
            new IntakeCoral(),
            new InstantCommand(() -> VisionManager.enableRightCam()),

            new InstantCommand(() -> VisionManager.disableLeftCam()),
            new MoveToAndExtend(ScoringPosition.L4, new FollowCurve(Tuning.AutoPaths.STATION_TO_K, Constants.REEF_POSITIONS.K.getRotation(), 0.75, 0.3)),
            Commands.waitSeconds(.2),
            new DropCoral(),
            new MoveToAndStow(new FollowCurve(Tuning.AutoPaths.K_TO_STATION, Constants.APRIL_TAGS[1].getRotationZ() + Math.PI, 0.9, 0.3, 0.075)),
            new IntakeCoral(),
            new InstantCommand(() -> VisionManager.enableLeftCam()),

            new InstantCommand(() -> VisionManager.disableRightCam()),
            new MoveToAndExtend(ScoringPosition.L4, new FollowCurve(Tuning.AutoPaths.STATION_TO_L, Constants.REEF_POSITIONS.L.getRotation(), 0.75, 0.3)),
            Commands.waitSeconds(.2),
            new DropCoral(),
            new MoveToAndStow(new FollowCurve(Tuning.AutoPaths.L_TO_STATION, Constants.APRIL_TAGS[1].getRotationZ() + Math.PI, 0.9, 0.3, 0.075)),
            new IntakeCoral()
        );
    }

    @AutoRoutine
    public SequentialCommandGroup fourPieceLeft(){
        SequentialCommandGroup auto = threeHalfPieceLeft();
        auto.addCommands(
            new InstantCommand(() -> VisionManager.enableRightCam()),
            new InstantCommand(() -> VisionManager.disableLeftCam()),
            new MoveToAndExtend(ScoringPosition.L4, new FollowCurve(Tuning.AutoPaths.STATION_TO_A, Constants.REEF_POSITIONS.A.getRotation(), 1.0, 0.3)),
            new DropCoral(),
            new MoveToScorePos(ScoringPosition.STOW)
        );
        return auto;
    }

    @AutoRoutine
    public SequentialCommandGroup fourPieceRight(){
        SequentialCommandGroup auto = threeHalfPieceRight();
        auto.addCommands(
            new InstantCommand(() -> VisionManager.enableLeftCam()),
            new InstantCommand(() -> VisionManager.disableRightCam()),
            new MoveToAndExtend(ScoringPosition.L4, new FollowCurve(Tuning.AutoPaths.STATION_TO_B, Constants.REEF_POSITIONS.B.getRotation(), 1.0, 0.3)),
            new DropCoral(),
            new MoveToScorePos(ScoringPosition.STOW)
        );
        return auto;
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
