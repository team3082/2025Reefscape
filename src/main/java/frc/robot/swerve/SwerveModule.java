package frc.robot.swerve;

import javax.print.DocFlavor.STRING;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Robot;
import frc.robot.swerve.sim.SwerveModuleSim;
import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;

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

    private SwerveModuleSim simModule = new SwerveModuleSim();

    private final double STEER_RATIO = (double) 150.0 / (double) 7.0; // TODO double check this value
    private final double DRIVE_RATIO = 1.0; // TODO double check this value

    public SwerveModule(int steerID, int driveID, double cancoderOffset, double x, double y) {
        steer = new TalonFX(steerID, "CANivore");
        drive = new TalonFX(driveID, "CANivore");
        absEncoder = new CANcoder(steerID, "CANivore");

        pos = new Vector2(x, y);

        // Configure encoders/PID
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();

        steerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.01;

        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; 

        steerConfig.Slot0.kP = 0.15;
        steerConfig.Slot0.kI = 0.0;
        steerConfig.Slot0.kD = 0.0;

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
        steerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

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
        steer.setControl(new PositionDutyCycle(radToRotSteer(targetAngle)));
        drive.setControl(new DutyCycleOut(inverted ? -targetSpeed : targetSpeed));
        
        // update simulation
        if (Robot.isSimulation()) {
            simModule.setAngle(targetAngle + (Math.PI / 2.0));
            simModule.setSpeed(targetSpeed);
            simModule.update();
        }
    }

    /** reset the internal encoder position to the absolute encoder position */
    public void resetSteerSensor() {
        double pos = absEncoder.getAbsolutePosition().getValueAsDouble() - cancoderOffset;
        steer.setPosition(-pos * STEER_RATIO);
        System.out.println("encoder pos " + pos);
        System.out.println("steer motor pos " + steer.getPosition().getValueAsDouble());
        System.out.println("steer ratio " + STEER_RATIO);
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
        // TODO Simplify angle using atan2

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
        System.out.println(destination);
        targetAngle = destination;
    }

    /** returns swerve wheel angle in radians */
    public double getSteerAngle() {
        if (Robot.isReal()) return rotToRadSteer(steer.getPosition().getValueAsDouble());
        else return simModule.getAngle() - Math.PI / 2.0;
    }

    private double lastSteerAngle = Double.NaN;

    /** TODO : FIX THIS FUNCTION
     * only call this once per frame
     */
    public double getSteerDelta() {
        if(lastSteerAngle == Double.NaN){
            lastSteerAngle = getSteerAngle();
            return 0.0;
        }
        double ret = getSteerAngle() - lastSteerAngle;
        lastSteerAngle = getSteerAngle();
        return ret;
    }

    /** TODO : FIX THIS FUNCTION
     * returns the current velocity of the drive motor in inches per second
     * @return
     */
    public double getDriveVelocity() {
        // TODO Recalculate
        double driveTimeConstant = 4 * Math.PI * 10;

        // if (RobotBase.isSimulation()) {
        //     return simDriveVel * driveTimeConstant;
        // }
        //the 10 is there to convert from units per 100ms to units per second
        if (Robot.isReal()) return drive.getVelocity().getValueAsDouble() * driveTimeConstant;
        else return (simModule.getSpeed() * driveTimeConstant * 4.0) * (inverted ? -1 : 1); // fudge factor
    }

    /** get position of the drive motor */
    public double getDrivePosition() {
        if (RobotBase.isReal()) return rotToRadDrive(drive.getPosition().getValueAsDouble());
        else return 0.0; // TODO add feature to sim
    }

    /** convert radians to internal motor rotations for the steer motor */
    private double radToRotSteer(double rad) {
        return (rad / (2.0 * Math.PI)) * STEER_RATIO;
    }

    /** convert internal motor rotations to radians for the steer motor */
    private double rotToRadSteer(double rot) {
        return (rot * (2.0 * Math.PI)) / STEER_RATIO;
    }

    /** convert internal motor rotations to radians for the drive motor */
    private double rotToRadDrive(double rot) {
        return (rot * (2.0 * Math.PI)) / DRIVE_RATIO;
    }
}
