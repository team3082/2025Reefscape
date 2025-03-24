package frc.robot.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Robot;
import frc.robot.swerve.sim.SwerveModuleSim;
import frc.robot.utils.Vector2;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.RobotBase;

public class SwerveModule {

    public TalonFX steer; // Steer Motor
    public TalonFX drive; // Drive Motor
    public CANcoder absEncoder; // CAN Coder

    public Vector2 pos; // x and y position of the module in relation to the drive base

    public double targetAngle; // Radians
    public double targetSpeed; // Percent Out
    public boolean inverted; // Drive Motor Inversion

    private final double cancoderOffset;

    public SwerveModuleSim simModule = new SwerveModuleSim();


    public SwerveModule(int steerID, int driveID, double cancoderOffset, double x, double y) {
        steer = new TalonFX(steerID, "CANivore");
        drive = new TalonFX(driveID, "CANivore");
        absEncoder = new CANcoder(steerID, "CANivore");

        pos = new Vector2(x, y);

        // Configure encoders/PID
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();

        steerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.01;

        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; 

        steerConfig.Slot0.kP = 0.5;
        steerConfig.Slot0.kI = 0.0001;
        steerConfig.Slot0.kD = 0.0025;

        steerConfig.MotionMagic.MotionMagicCruiseVelocity = 40000;
        steerConfig.MotionMagic.MotionMagicAcceleration = 40000;

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; 
        drive.getConfigurator().setPosition(0);
        driveConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;

        driveConfig.Slot0.kP = 0.5;
        driveConfig.Slot0.kI = 0.02;
        driveConfig.Slot0.kD = 0.2;

        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
        driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // CHANGED THIS MARKING HERE

        CANcoderConfiguration canConfig = new CANcoderConfiguration();
        canConfig.MagnetSensor.MagnetOffset = 0;
        canConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

        absEncoder.getConfigurator().apply(canConfig);

        driveConfig.CurrentLimits.SupplyCurrentLimit = 39;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        steerConfig.CurrentLimits.SupplyCurrentLimit = 30;
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        this.cancoderOffset = cancoderOffset;

        inverted = false;

        drive.getConfigurator().apply(driveConfig);
        steer.getConfigurator().apply(steerConfig);
        
        resetSteerSensor();
    }

    /** update swerve module, set motor positions/speeds only call in SwerveManager.update() */
    public void update() {
        // apply motor control
        steer.setControl(new PositionDutyCycle(radToRotSteer(targetAngle + (Math.PI / 2.0))));
        drive.setControl(new DutyCycleOut(inverted ? -targetSpeed : targetSpeed));
        
        // update simulation

        if (Robot.isSimulation()) {
            simModule.setAngle(targetAngle);
            simModule.setSpeed(targetSpeed);
            simModule.update(inverted);
        }
    }

    /** reset the internal encoder position to the absolute encoder position */
    public void resetSteerSensor() {
        double pos = absEncoder.getAbsolutePosition().getValueAsDouble() - cancoderOffset;
        steer.setPosition(pos * Constants.Swerve.STEER_RATIO);
    }

    /** set target drive speed */
    public void drive(double power) {
        targetSpeed = power;
    }

    /** sets target steer position in radians and drive inversion - btw this shit is atrocious */
    public void rotateToRad(double angle) {
        double motorPos;
        
        motorPos = getSteerAngle();

        // The number of full rotations the motor has made
        double numRot = Math.floor(motorPos / (2.0 * Math.PI));

        // The target motor position dictated by the joystick, in rotations
        double joystickTarget = (numRot * 2.0 * Math.PI) + angle;
        double joystickTargetPlus = joystickTarget + (2.0 * Math.PI);
        double joystickTargetMinus = joystickTarget - (2.0 * Math.PI);

        // The true destination for the motor to rotate to
        double destination;

        // Determine if, based on the current motor position, it should stay in the same
        // rotation, enter the next, or return to the previous.
        if (Math.abs(joystickTarget - motorPos) < Math.abs(joystickTargetPlus - motorPos)
                && Math.abs(joystickTarget - motorPos) < Math.abs(joystickTargetMinus - motorPos)) {
            destination = joystickTarget;
        } else if (Math.abs(joystickTargetPlus - motorPos) < Math.abs(joystickTargetMinus - motorPos)) {
            destination = joystickTargetPlus;
        } else {
            destination = joystickTargetMinus;
        }


        // If the target position is farther than a quarter rotation away from the
        // current position, invert its direction instead of rotating it the full
        // distance
        if (Math.abs(destination - motorPos) > (Math.PI / 2.0)) {
            inverted = true;
            if (destination > motorPos)
                destination -= Math.PI;
            else
                destination += Math.PI;
        } else {
            inverted = false;
        }
        
        targetAngle = destination;
    }

    /** returns swerve wheel angle in radians */
    public double getSteerAngle() {
        if (Robot.isReal()) return rotToRadSteer(steer.getPosition().getValueAsDouble()) - (Math.PI / 2.0);
        else return simModule.getAngle();
    }


    /**
     * returns the current velocity of the drive motor in inches per second
     * @return
     */
    public double getDriveVelocity() {
        double driveTimeConstant = 2.0 * Math.PI * 2.0;

        if (Robot.isReal()) return (drive.getVelocity().getValueAsDouble() / Constants.Swerve.STEER_RATIO) * driveTimeConstant;
        else return (simModule.getSpeed() * driveTimeConstant * 10) * (inverted ? -1 : 1); // fudge factor
    }

    /** get position of the drive motor */
    public double getDrivePosition() {
        if (RobotBase.isReal()) return rotToRadDrive(drive.getPosition().getValueAsDouble() * 2.0);
        else return rotToRadDrive(simModule.getDrivePosition());
    }

    /** convert radians to internal motor rotations for the steer motor */
    private double radToRotSteer(double rad) {
        return (rad / (2.0 * Math.PI)) * Constants.Swerve.STEER_RATIO;
    }

    /** convert internal motor rotations to radians for the steer motor */
    private double rotToRadSteer(double rot) {
        return (rot * (2.0 * Math.PI)) / Constants.Swerve.STEER_RATIO;
    }

    /** convert internal motor rotations to radians for the drive motor */
    private double rotToRadDrive(double rot) {
        return (rot * (2.0 * Math.PI)) / Constants.Swerve.DRIVE_RATIO;
    }
}
