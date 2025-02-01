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

import edu.wpi.first.wpilibj.RobotBase;

public class SwerveModule {

    public TalonFX steer;
    public TalonFX drive;
    public CANcoder absEncoder;

    public Vector2 pos;

    private double targetAngle; // Radians
    private double targetSpeed; // Percent Out
    private boolean inverted;

    private final double cancoderOffset;

    // NEW SIMULATION
    private SwerveModuleSim simModule = new SwerveModuleSim();

    // OLD SIMULATION
    private double simSteerAng;
    private double simDriveVel;

    private final double STEER_RATIO = 150.0 / 7.0; // TODO double check this value
    private final double DRIVE_RATIO = 1.0; // TODO double check this value

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

    public void update() {
        // apply motor control
        steer.setControl(new PositionDutyCycle(radToRotSteer(targetAngle) + (Math.PI / 2.0)));
        drive.setControl(new DutyCycleOut(inverted ? -targetSpeed : targetSpeed));
        

        // update simulation
        if (Robot.isSimulation()) {
            simModule.setAngle(targetAngle);
            simModule.setSpeed(targetSpeed);
            simModule.update();
        }
    }

    public void resetSteerSensor() {
        double pos = absEncoder.getAbsolutePosition().getValueAsDouble() - cancoderOffset;
        pos = pos / 360.0;
        steer.getConfigurator().setPosition(pos);
    }

    public void drive(double power) {
        // OLD CODE
        // drive.set(power);
        // simDriveVel = power * (inverted ? -1.0 : 1.0);

        // NEW CODE
        targetSpeed = power;
    }

    // Rotates to angle given in radians
    public void rotateToRad(double angle) {
        // OLD CODE
        // rotate((angle - Math.PI / 2) / (2 * Math.PI));
        
        // NEW CODE
        double currentAngle = getSteerAngle();
        double deltaAngle = angle - currentAngle;

        // Normalize the delta angle to be within the range of -PI to PI
        deltaAngle = (deltaAngle + Math.PI) % (2 * Math.PI) - Math.PI;

        // If the delta angle is greater than 90 degrees, invert the drive motor and adjust the target angle
        if (Math.abs(deltaAngle) > Math.PI / 2.0) {
            deltaAngle -= Math.signum(deltaAngle) * Math.PI;
            inverted = !inverted;
        }

        // Set the target angle
        targetAngle = currentAngle + deltaAngle;
    }

    // Returns an angle in radians
    public double getSteerAngle() {
        // OLD CODE
        // if (RobotBase.isSimulation()) {
        //     return simSteerAng * Math.PI * 2 + Math.PI / 2;
        // }
        
        // return steer.getPosition().getValueAsDouble() * Math.PI * 2 + Math.PI / 2;

        // NEW CODE
        return rotToRadSteer(steer.getPosition().getValueAsDouble()) + Math.PI / 2.0;
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

    private double radToRotSteer(double rad) {
        return (rad / (2.0 * Math.PI)) * STEER_RATIO;
    }

    private double rotToRadSteer(double rot) {
        return (rot * (2.0 * Math.PI)) / STEER_RATIO;
    }

    private double radToRotDrive(double rad) {
        return (rad / (2.0 * Math.PI)) * DRIVE_RATIO;
    }

    private double rotToRadDrive(double rot) {
        return (rot * (2.0 * Math.PI)) / DRIVE_RATIO;
    }
}
