package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.Constants.EndEffector;
import frc.robot.subsystems.ScoringManager;
import frc.robot.subsystems.visualizer.ElevatorVisualizer;
import frc.robot.subsystems.visualizer.EndEffectorVisualizer;

public class Telemetry {
    private static ShuffleboardTab robotTab = Shuffleboard.getTab("Robot Views");
    private static ShuffleboardTab scoringManagerTab = Shuffleboard.getTab("Scoring Manager");
    private static ShuffleboardTab endEffectorTab = Shuffleboard.getTab("End Effector");
    private static ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");

    // Views
    private static Field2d fieldView = new Field2d();
    public static Mechanism2d subsytemView = new Mechanism2d(50, 120);

    // Logging
    // Scoring Manager
    private static final GenericEntry SCORING_TRANSITORY_STATE = scoringManagerTab.add("transitory state", ScoringManager.transitoryState.name()).getEntry();
    private static final GenericEntry SCORING_POSITION = scoringManagerTab.add("scoring position", ScoringManager.scoringPosition.name()).getEntry();

    // End Effector
    private static final GenericEntry END_EFFECTOR_TARGET_ANGLE = endEffectorTab.add("target angle", ScoringManager.endEffector.targetAngle).getEntry();
    private static final GenericEntry END_EFFECTOR_CURRENT_ANGLE = endEffectorTab.add("current angle", ScoringManager.endEffector.getPivotAngle()).getEntry();
    private static final GenericEntry END_EFFECTOR_WHEEL_SPEED = endEffectorTab.add("wheel speed", ScoringManager.endEffector.intakeState.speed).getEntry();

    // Elevator
    private static final GenericEntry ELEVATOR_TARGET_POSITION = elevatorTab.add("target position", ScoringManager.elevator.targetHeight).getEntry();
    private static final GenericEntry ELEVATOR_CURRENT_POSITION = elevatorTab.add("current position", ScoringManager.elevator.getElevatorHeight()).getEntry();

    public static void init() {
        robotTab.add("Field", fieldView);
        robotTab.add("Subsystem View", subsytemView);

        ElevatorVisualizer.init();
        EndEffectorVisualizer.init();
    }

    public static void update() {
        SCORING_TRANSITORY_STATE.setString(ScoringManager.transitoryState.name());
        SCORING_POSITION.setString(ScoringManager.scoringPosition.name());

        END_EFFECTOR_TARGET_ANGLE.setDouble(ScoringManager.endEffector.targetAngle);
        END_EFFECTOR_CURRENT_ANGLE.setDouble(ScoringManager.endEffector.getPivotAngle());
        END_EFFECTOR_WHEEL_SPEED.setDouble(ScoringManager.endEffector.intakeState.speed);

        ELEVATOR_TARGET_POSITION.setDouble(ScoringManager.elevator.targetHeight);
        ELEVATOR_CURRENT_POSITION.setDouble(ScoringManager.elevator.getElevatorHeight());

        ElevatorVisualizer.update();
        EndEffectorVisualizer.update();
    }
}
