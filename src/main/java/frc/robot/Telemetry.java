package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.auto.Auto;
import frc.robot.subsystems.ScoringManager;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.subsystems.visualizer.CoralVisualizer;
import frc.robot.subsystems.sim.AlgaeSim;
import frc.robot.subsystems.visualizer.AlgaeVisualizer;
import frc.robot.subsystems.visualizer.ElevatorVisualizer;
import frc.robot.subsystems.visualizer.EndEffectorVisualizer;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePosition;
import frc.robot.swerve.visualizer.SwerveBaseVisualizer;
import frc.robot.swerve.SwervePID;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.Vector2;
import frc.robot.auto.Auto;

/*
 * handles telemetry for the robot
 * reads values from subsystems and updates tables and visualizers
 * includes the field view and subsystem views
 */
public class Telemetry {
    private static ShuffleboardTab robotTab = Shuffleboard.getTab("Robot Views");
    private static ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
    private static ShuffleboardTab scoringManagerTab = Shuffleboard.getTab("Scoring Manager");
    private static ShuffleboardTab endEffectorTab = Shuffleboard.getTab("End Effector");
    private static ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");

    // Views
    private static Field2d fieldView = new Field2d();
    public static Mechanism2d subsytemView = new Mechanism2d(65, 120);
    public static Mechanism2d swerveView = new Mechanism2d(60, 60);

    

    // Logging
    // Swerve
    private static final GenericEntry SWERVE_MOD_1_ANGLE = swerveTab.add("Swerve Module 1 Angle", SwerveManager.mods[0].getSteerAngle()).getEntry();
    private static final GenericEntry SWERVE_MOD_1_SPEED = swerveTab.add("Swerve Module 1 Speed", SwerveManager.mods[0].getDriveVelocity()).getEntry();
    private static final GenericEntry SWERVE_MOD_1_TARGET_ANGLE = swerveTab.add("Swerve Module 1 Target Angle", SwerveManager.mods[0].targetAngle).getEntry();
    private static final GenericEntry SWERVE_MOD_1_TARGET_SPEED = swerveTab.add("Swerve Module 1 Target Speed", SwerveManager.mods[0].targetSpeed).getEntry();
    private static final GenericEntry SWERVE_MOD_1_INVERTED = swerveTab.add("Swerve Module 1 Inverted", SwerveManager.mods[0].inverted).getEntry();

    private static final GenericEntry SWERVE_MOD_2_ANGLE = swerveTab.add("Swerve Module 2 Angle", SwerveManager.mods[1].getSteerAngle()).getEntry();
    private static final GenericEntry SWERVE_MOD_2_SPEED = swerveTab.add("Swerve Module 2 Speed", SwerveManager.mods[1].getDriveVelocity()).getEntry();
    private static final GenericEntry SWERVE_MOD_2_TARGET_ANGLE = swerveTab.add("Swerve Module 2 Target Angle", SwerveManager.mods[1].targetAngle).getEntry();
    private static final GenericEntry SWERVE_MOD_2_TARGET_SPEED = swerveTab.add("Swerve Module 2 Target Speed", SwerveManager.mods[1].targetSpeed).getEntry();
    private static final GenericEntry SWERVE_MOD_2_INVERTED = swerveTab.add("Swerve Module 2 Inverted", SwerveManager.mods[1].inverted).getEntry();

    private static final GenericEntry SWERVE_MOD_3_ANGLE = swerveTab.add("Swerve Module 3 Angle", SwerveManager.mods[2].getSteerAngle()).getEntry();
    private static final GenericEntry SWERVE_MOD_3_SPEED = swerveTab.add("Swerve Module 3 Speed", SwerveManager.mods[2].getDriveVelocity()).getEntry();
    private static final GenericEntry SWERVE_MOD_3_TARGET_ANGLE = swerveTab.add("Swerve Module 3 Target Angle", SwerveManager.mods[2].targetAngle).getEntry();
    private static final GenericEntry SWERVE_MOD_3_TARGET_SPEED = swerveTab.add("Swerve Module 3 Target Speed", SwerveManager.mods[2].targetSpeed).getEntry();
    private static final GenericEntry SWERVE_MOD_3_INVERTED = swerveTab.add("Swerve Module 3 Inverted", SwerveManager.mods[2].inverted).getEntry();

    private static final GenericEntry SWERVE_MOD_4_ANGLE = swerveTab.add("Swerve Module 4 Angle", SwerveManager.mods[3].getSteerAngle()).getEntry();
    private static final GenericEntry SWERVE_MOD_4_SPEED = swerveTab.add("Swerve Module 4 Speed", SwerveManager.mods[3].getDriveVelocity()).getEntry();
    private static final GenericEntry SWERVE_MOD_4_TARGET_ANGLE = swerveTab.add("Swerve Module 4 Target Angle", SwerveManager.mods[3].targetAngle).getEntry();
    private static final GenericEntry SWERVE_MOD_4_TARGET_SPEED = swerveTab.add("Swerve Module 4 Target Speed", SwerveManager.mods[3].targetSpeed).getEntry();
    private static final GenericEntry SWERVE_MOD_4_INVERTED = swerveTab.add("Swerve Module 4 Inverted", SwerveManager.mods[3].inverted).getEntry();

    // // Scoring Manager
    // private static final GenericEntry SCORING_TRANSITORY_STATE = scoringManagerTab.add("transitory state", ScoringManager.transitoryState.name()).getEntry();
    // private static final GenericEntry SCORING_POSITION = scoringManagerTab.add("scoring position", ScoringManager.scoringPosition.name()).getEntry();

    // // End Effector
    // private static final GenericEntry END_EFFECTOR_TARGET_ANGLE = endEffectorTab.add("target angle", ScoringManager.endEffector.targetAngle).getEntry();
    // private static final GenericEntry END_EFFECTOR_CURRENT_ANGLE = endEffectorTab.add("current angle", ScoringManager.endEffector.getPivotAngle()).getEntry();
    // private static final GenericEntry END_EFFECTOR_WHEEL_SPEED = endEffectorTab.add("wheel speed", ScoringManager.endEffector.intakeState.targetSpeed).getEntry();

    // // Elevator
    // private static final GenericEntry ELEVATOR_TARGET_POSITION = elevatorTab.add("target position", ScoringManager.elevator.targetHeight).getEntry();
    // private static final GenericEntry ELEVATOR_CURRENT_POSITION = elevatorTab.add("current position", ScoringManager.elevator.getElevatorHeight()).getEntry();

    public static void init() {
        robotTab.add("Field", fieldView);
        // robotTab.add("Subsystem View", subsytemView);
        robotTab.add("Swerve View", swerveView);
        robotTab.add("Coral Operator View", CoralVisualizer.getMechanism2d());

        // ElevatorVisualizer.init();
        // EndEffectorVisualizer.init();
        SwerveBaseVisualizer.init();
        AlgaeVisualizer.init();
        robotTab.addString("Position", () -> SwervePosition.getPosition().toString());
        robotTab.addString("PID Dest Position", () -> SwervePID.getDest().toString());
        robotTab.addString("Dest Error", () -> SwervePID.getError().toString());
        robotTab.addString("Update Output Vel", () -> SwervePID.updateOutputVel().toString());
        robotTab.addString("xPID Error", () -> SwervePID.getError().toString());
        robotTab.addDouble("Rot Error", () -> SwervePID.getRotationError());

        robotTab.addBoolean("AtDest", () -> SwervePID.atDest());
        robotTab.addBoolean("AtRot", () -> SwervePID.atRot());

        robotTab.addDouble("Rotation Rads", () -> Pigeon.getRotationRad());
        // robotTab.add(Auto.getAutoSelector());

        Logger.recordOutput("Error", SwervePID.getError().toString());


        robotTab.add("Auto Selector", Auto.routineManager.autoSelector);
    }

    public static void update() {
        updateField();
        updateSwerve();
        // updateScoring();
        
    }

    private static void updateSwerve() {
        SWERVE_MOD_1_ANGLE.setDouble(SwerveManager.mods[0].getSteerAngle());
        SWERVE_MOD_1_SPEED.setDouble(SwerveManager.mods[0].getDriveVelocity());
        SWERVE_MOD_1_TARGET_ANGLE.setDouble(SwerveManager.mods[0].targetAngle);
        SWERVE_MOD_1_TARGET_SPEED.setDouble(SwerveManager.mods[0].targetSpeed);
        SWERVE_MOD_1_INVERTED.setBoolean(SwerveManager.mods[0].inverted);

        SWERVE_MOD_2_ANGLE.setDouble(SwerveManager.mods[1].getSteerAngle());
        SWERVE_MOD_2_SPEED.setDouble(SwerveManager.mods[1].getDriveVelocity());
        SWERVE_MOD_2_TARGET_ANGLE.setDouble(SwerveManager.mods[1].targetAngle);
        SWERVE_MOD_2_TARGET_SPEED.setDouble(SwerveManager.mods[1].targetSpeed);
        SWERVE_MOD_2_INVERTED.setBoolean(SwerveManager.mods[1].inverted);

        SWERVE_MOD_3_ANGLE.setDouble(SwerveManager.mods[2].getSteerAngle());
        SWERVE_MOD_3_SPEED.setDouble(SwerveManager.mods[2].getDriveVelocity());
        SWERVE_MOD_3_TARGET_ANGLE.setDouble(SwerveManager.mods[2].targetAngle);
        SWERVE_MOD_3_TARGET_SPEED.setDouble(SwerveManager.mods[2].targetSpeed);
        SWERVE_MOD_3_INVERTED.setBoolean(SwerveManager.mods[2].inverted);

        SWERVE_MOD_4_ANGLE.setDouble(SwerveManager.mods[3].getSteerAngle());
        SWERVE_MOD_4_SPEED.setDouble(SwerveManager.mods[3].getDriveVelocity());
        SWERVE_MOD_4_TARGET_ANGLE.setDouble(SwerveManager.mods[3].targetAngle);
        SWERVE_MOD_4_TARGET_SPEED.setDouble(SwerveManager.mods[3].targetSpeed);
        SWERVE_MOD_4_INVERTED.setBoolean(SwerveManager.mods[3].inverted);

        SwerveBaseVisualizer.update();
    }

    /**
     * Updates the simulated field in shuffleboard based on SwervePosition
     */
    private static void updateField(){
        
        // Current position adjusted to be in the center of the field at (0,0)
        Pose2d currentPose = new Pose2d(
            SwervePosition.getPosition().y/Constants.METERSTOINCHES + 8.27,
            SwervePosition.getPosition().x/Constants.METERSTOINCHES + 4.01,
            Rotation2d.fromRadians(Pigeon.getRotationRad())
        );
        fieldView.setRobotPose(currentPose);
        SmartDashboard.putData(fieldView);
    }

    /**
     * Updates the Mech2d values of the simulated scoring subsystems
     */
    // private static void updateScoring(){
    //     // update table values
    //     SCORING_TRANSITORY_STATE.setString(ScoringManager.transitoryState.name());
    //     SCORING_POSITION.setString(ScoringManager.scoringPosition.name());

    //     END_EFFECTOR_TARGET_ANGLE.setDouble(ScoringManager.endEffector.targetAngle);
    //     END_EFFECTOR_CURRENT_ANGLE.setDouble(ScoringManager.endEffector.getPivotAngle());
    //     END_EFFECTOR_WHEEL_SPEED.setDouble(ScoringManager.endEffector.intakeState.targetSpeed);

    //     ELEVATOR_TARGET_POSITION.setDouble(ScoringManager.elevator.targetHeight);
    //     ELEVATOR_CURRENT_POSITION.setDouble(ScoringManager.elevator.getElevatorHeight());

    //     ElevatorVisualizer.update();
    //     EndEffectorVisualizer.update();
    // }
}
