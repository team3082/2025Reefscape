package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.Auto;
import frc.robot.subsystems.ScoringManager;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.subsystems.visualizer.ClimberVisualizer;
import frc.robot.subsystems.visualizer.ElevatorVisualizer;
import frc.robot.subsystems.visualizer.EndEffectorVisualizer;
import frc.robot.subsystems.visualizer.FunnelVisualizer;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePosition;
import frc.robot.swerve.visualizer.SwerveBaseVisualizer;
import frc.robot.swerve.SwervePID;

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
    public static LoggedMechanism2d subsystemView = new LoggedMechanism2d(65/Constants.METERSTOINCHES, 120/Constants.METERSTOINCHES);
    //public static Mechanism2d subsytemView = new Mechanism2d(65, 120);
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

    // Scoring Manager
    private static final GenericEntry SCORING_TRANSITORY_STATE = scoringManagerTab.add("transitory state", ScoringManager.transitoryState.name()).getEntry();
    private static final GenericEntry SCORING_POSITION = scoringManagerTab.add("scoring position", ScoringManager.scoringPosition.name()).getEntry();

    // End Effector
    private static final GenericEntry END_EFFECTOR_TARGET_ANGLE = endEffectorTab.add("target angle", ScoringManager.endEffector.targetAngle).getEntry();
    private static final GenericEntry END_EFFECTOR_CURRENT_ANGLE = endEffectorTab.add("current angle", ScoringManager.endEffector.getPivotAngle()).getEntry();
    private static final GenericEntry END_EFFECTOR_WHEEL_SPEED = endEffectorTab.add("wheel speed", ScoringManager.endEffector.intakeState.targetSpeed).getEntry();

    // Elevator
    private static final GenericEntry ELEVATOR_TARGET_POSITION = elevatorTab.add("target position", ScoringManager.elevator.targetHeight).getEntry();
    private static final GenericEntry ELEVATOR_CURRENT_POSITION = elevatorTab.add("current position", ScoringManager.elevator.getElevatorHeight()).getEntry();

    public static void init() {
        robotTab.add("Field View", fieldView);
        robotTab.add("Subsystem View", subsystemView);
        robotTab.add("Swerve View", swerveView);

        ElevatorVisualizer.init();
        EndEffectorVisualizer.init();
        SwerveBaseVisualizer.init();
        ClimberVisualizer.init();
        FunnelVisualizer.init();
        
        robotTab.addString("Position", () -> SwervePosition.getPosition().toString());
        robotTab.addString("PID Dest Position", () -> SwervePID.getDest().toString());
        // robotTab.addString("Dest Error", () -> SwervePID.getError().toString());
        // robotTab.addString("Update Output Vel", () -> SwervePID.updateOutputVel().toString());
        // robotTab.addString("xPID Error", () -> SwervePID.getError().toString());
        // robotTab.addDouble("Rot Error", () -> SwervePID.getRotationError());

        // robotTab.addBoolean("AtDest", () -> SwervePID.atDest());
        // robotTab.addBoolean("AtRot", () -> SwervePID.atRot());

        // robotTab.addDouble("Rotation Rads", () -> Pigeon.getRotationRad());
        // robotTab.add(Auto.getAutoSelector());

        

        robotTab.add("Auto Selector", Auto.routineManager.autoSelector);
    }

    public static void update() {
        updateField();
        updateSwerve();
        updateScoring();
        logValues();
    }

    private static void logValues(){
        Logger.recordOutput("Robot/SwervePID/Error", SwervePID.getError().toString());
        Logger.recordOutput("Robot/SwervePID/Rot Error", SwervePID.getRotationError());
        Logger.recordOutput("Robot/SwervePID/Destination", new Pose2d(SwervePID.getDest().x/Constants.METERSTOINCHES + 8.78,
                                                                          SwervePID.getDest().y/Constants.METERSTOINCHES + 4.01,
                                                                          Rotation2d.fromRadians(SwervePID.getTargetRot() + Math.PI/2)));
        Logger.recordOutput("Robot/SwervePID/At Dest", SwervePID.atDest());
        Logger.recordOutput("Robot/SwervePID/At Dest/x", SwervePID.xPID.atSetpoint());
        Logger.recordOutput("Robot/SwervePID/At Dest/y", SwervePID.yPID.atSetpoint());
        Logger.recordOutput("Robot/SwervePID/At Rot", SwervePID.atRot());
        Logger.recordOutput("Robot/Swerve Position", SwervePosition.getPosition().toString());
        Logger.recordOutput("Robot/Swerve Position/x", SwervePosition.getPosition().x);
        Logger.recordOutput("Robot/Swerve Position/y", SwervePosition.getPosition().y);
        Logger.recordOutput("Robot/Swerve Position/rot", Pigeon.getRotationRad());

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
    private static void updateField() {
        // Current position adjusted to be in the center of the field at (0,0)
        
        Pose2d currentPose = new Pose2d(
            SwervePosition.getPosition().x /Constants.METERSTOINCHES + 8.78,
            SwervePosition.getPosition().y/Constants.METERSTOINCHES + 4.01,
            Rotation2d.fromRadians(Pigeon.getRotationRad() + Robot.getAllianceMultiplier() * Math.PI / 2.0)
        );
        fieldView.setRobotPose(currentPose);
        SmartDashboard.putData(fieldView);

        Logger.recordOutput("Robot/Swerve/Field Pose", currentPose);
    }

    /**
     * Updates the Mech2d values of the simulated scoring subsystems
     */
    private static void updateScoring(){
        // update table values
        SCORING_TRANSITORY_STATE.setString(ScoringManager.transitoryState.name());
        SCORING_POSITION.setString(ScoringManager.scoringPosition.name());

        END_EFFECTOR_TARGET_ANGLE.setDouble(ScoringManager.endEffector.targetAngle);
        END_EFFECTOR_CURRENT_ANGLE.setDouble(ScoringManager.endEffector.getPivotAngle());
        END_EFFECTOR_WHEEL_SPEED.setDouble(ScoringManager.endEffector.intakeState.targetSpeed);

        ELEVATOR_TARGET_POSITION.setDouble(ScoringManager.elevator.targetHeight);
        ELEVATOR_CURRENT_POSITION.setDouble(ScoringManager.elevator.getElevatorHeight());

        ElevatorVisualizer.update();
        EndEffectorVisualizer.update();
        ClimberVisualizer.update();
        FunnelVisualizer.update();

        Logger.recordOutput("Robot/Subsystem View", subsystemView);
    }
}
