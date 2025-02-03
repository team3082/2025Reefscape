package frc.robot.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.utils.Vector2;

import edu.wpi.first.wpilibj.RobotBase;

public class SwerveModule {

    public TalonFX steer;
    public TalonFX drive;
    public CANcoder absEncoder;

    public Vector2 pos;

    private boolean inverted;

    private final double cancoderOffset;

    private double simSteerAng;
    private double simDriveVel;


    public SwerveModule(int steerID, int driveID, double cancoderOffset, double x, double y) {
        steer = new TalonFX(steerID, "CANivore");
        drive = new TalonFX(driveID, "CANivore");
        absEncoder = new CANcoder(steerID, "CANivore");

        pos = new Vector2(x, y);

        // Configure encoders/PID
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();

        steerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;

        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; 

        steerConfig.Slot0.kP = 0.4;
        steerConfig.Slot0.kI = 0.0;
        steerConfig.Slot0.kD = 0.2;

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

    public void resetSteerSensor() {
        double pos = absEncoder.getAbsolutePosition().getValueAsDouble() - cancoderOffset;
        pos = pos / 360.0;
        steer.getConfigurator().setPosition(pos);
    }

    public void drive(double power) {
        drive.set(power);

        simDriveVel = power * (inverted ? -1.0 : 1.0);
    }

    // Rotates to angle given in radians
    public void rotateToRad(double angle) {
        rotate((angle - Math.PI / 2) / (2 * Math.PI));
    }

    // Rotates to a position given in rotations
    public void rotate(double toAngle) {
        double motorPos;
        
        if (RobotBase.isSimulation())
            motorPos = simSteerAng;
        else 
            motorPos = steer.getRotorPosition().getValueAsDouble();

        // The number of full rotations the motor has made
        int numRot = (int) Math.floor(motorPos);

        // The target motor position dictated by the joystick, in rotations
        double joystickTarget = numRot + toAngle;
        double joystickTargetPlus = joystickTarget + 1.0;
        double joystickTargetMinus = joystickTarget - 1.0;

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
        if (Math.abs(destination - motorPos) > 0.25) {
            inverted = true;
            if (destination > motorPos)
                destination -= 0.5;
            else
                destination += 0.5;
        } else {
            inverted = false;
        }

        steer.setPosition(destination);
        simSteerAng = destination;
    }

    // Returns an angle in radians
    public double getSteerAngle() {
        if (RobotBase.isSimulation()) {
            return simSteerAng * Math.PI * 2 + Math.PI / 2;
        }
        
        return steer.getPosition().getValueAsDouble() * Math.PI * 2 + Math.PI / 2;
    }

    private double lastSteerAngle = Double.NaN;

    /**
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

    /**
     * returns the current velocity of the drive motor in inches per second
     * @return
     */
    public double getDriveVelocity() {
        // TODO Recalculate
        double driveTimeConstant = 4 * Math.PI * 10;

        if (RobotBase.isSimulation()) {
            return simDriveVel * driveTimeConstant;
        }
        //the 10 is there to convert from units per 100ms to units per second
        return drive.getVelocity().getValueAsDouble() * driveTimeConstant;
        
    }

    private double simDrivePosition;

    public double getDrivePosition(){
        if(RobotBase.isReal()){
            return drive.getPosition().getValueAsDouble() * (4 * Math.PI);
        }
        //IF in sim

        //TODO this will only ever work with a 7 ms loop
        simDrivePosition += getDriveVelocity() * 0.07 /10;//added fudge factor so it is somewhat accurate to real life
        return simDrivePosition;
    }
}
