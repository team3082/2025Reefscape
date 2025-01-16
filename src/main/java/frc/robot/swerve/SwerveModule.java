package frc.robot.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.utils.Vector2;

import edu.wpi.first.wpilibj.RobotBase;

@SuppressWarnings("removal")
public class SwerveModule {

    public TalonFX steer;
    public TalonFX drive;
    public CANCoder absEncoder;

    public Vector2 pos;

    private boolean inverted;

    private final double cancoderOffset;

    private double simSteerAng;
    private double simDriveVel;

    // should probably only use one for both drive & steer but I'll keep both for now
    VoltageOut requestToApplyDrive;
    VoltageOut requestToApplySteer;


    public SwerveModule(int steerID, int driveID, double cancoderOffset, double x, double y) {
        steer = new TalonFX(steerID, "CANivore");
        drive = new TalonFX(driveID, "CANivore");
        absEncoder = new CANCoder(steerID, "CANivore");

        pos = new Vector2(x, y);

        // Configure encoders/PID

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steer.getConfigurator().apply(new TalonFXConfiguration());


        //steer.configNeutralDeadband(0.001, 30); -- v5 to v6
        steerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;

        //steer.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; 

        steerConfig.Slot0.kP = 0.4;
        steerConfig.Slot0.kI = 0.0;
        steerConfig.Slot0.kD = 0.2;
        /* steer.configMotionCruiseVelocity(40000, 30);
        steer.configMotionAcceleration(40000, 30); -- v5 to v6 */ 
        steerConfig.MotionMagic.MotionMagicCruiseVelocity = 40000;
        steerConfig.MotionMagic.MotionMagicAcceleration = 40000;


        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        drive.getConfigurator().apply(new TalonFXConfiguration());

        //drive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; 
        //drive.setSelectedSensorPosition(0);
        drive.getConfigurator().setPosition(0);
        driveConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;

        // driveConfig.Slot0.kF = 0.0;
        driveConfig.Slot0.kP = 0.5;
        driveConfig.Slot0.kI = 0.02;
        driveConfig.Slot0.kD = 0.2;


        
        drive.setInverted(true);
        steer.setInverted(true);
        //drive.setNeutralMode(NeutralMode.Brake);
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
        //steer.setNeutralMode(NeutralMode.Brake);
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        absEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        absEncoder.configMagnetOffset(0);
        absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        // SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration(true, 39, 39, 0 );
        driveConfig.CurrentLimits.SupplyCurrentLimit = 39;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

       // -----
       // drive.configVoltageCompSaturation(11.9);
       // drive.enableVoltageCompensation(true);

       // when in main robot code, set withOutput of drive motor to 11.9 (part of pheonix v6 migration)
       requestToApplyDrive = new VoltageOut(0);

        // -----
        // SupplyCurrentLimitConfiguration steerCurrentLimit = new SupplyCurrentLimitConfiguration(true, 30, 30, 0 );
        // steer.configSupplyCurrentLimit(steerCurrentLimit);
        steerConfig.CurrentLimits.SupplyCurrentLimit = 30;
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // -----
        //steer.configVoltageCompSaturation(11.9);
        //steer.enableVoltageCompensation(true);

        // when in main robot code, set withOutput of steer motor to 11.9 (part of pheonix v6 migration)
        requestToApplySteer = new VoltageOut(0);

        this.cancoderOffset = cancoderOffset;

        inverted = false;

        resetSteerSensor();
    }

    public void resetSteerSensor() {
        double pos = absEncoder.getAbsolutePosition() - cancoderOffset;
        pos = pos / 360.0;
        //steer.setSelectedSensorPosition(pos);
        steer.getConfigurator().setPosition(pos);
    }

    public void drive(double power) {
        // drive.set(TalonFXControlMode.PercentOutput, power * (inverted ? -1.0 : 1.0));
        drive.setControl(requestToApplyDrive.withOutput(11.9));

        simDriveVel = power * (inverted ? -1.0 : 1.0);
    }

    // Rotates to angle given in radians
    public void rotateToRad(double angle) {
        rotate((angle - Math.PI / 2) / (2 * Math.PI));
    }

    // Rotates to a position given in rotations
    @SuppressWarnings("rawtypes")
    public void rotate(double toAngle) {
        double motorPos;
        StatusSignal steerPosition = steer.getRotorPosition();
        
        if (RobotBase.isSimulation())
            motorPos = simSteerAng;
        else 
            // motorPos = steer.getSelectedSensorPosition();
            motorPos = steerPosition.getValueAsDouble();

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

        // steer.set(TalonFXControlMode.Position, destination);
        // can someone please check this, I don't know what voltage to put here at the moment so I put a placeholder
        steer.setControl(requestToApplySteer.withOutput(5.0));

        simSteerAng = destination;
    }

    // Returns an angle in radians
    @SuppressWarnings("rawtypes")
    public double getSteerAngle() {
        if (RobotBase.isSimulation()) {
            return simSteerAng * Math.PI * 2 + Math.PI / 2;
        }
        StatusSignal pos = steer.getPosition();
        return pos.getValueAsDouble() * Math.PI * 2 + Math.PI / 2;
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
    @SuppressWarnings("rawtypes")
    public double getDriveVelocity() {
        if (RobotBase.isSimulation()) {
            return simDriveVel * 10 * (4 * Math.PI);
        }
        //the 10 is there to convert from units per 100ms to units per second
        StatusSignal vel = drive.getVelocity();
        return vel.getValueAsDouble() * 10 * (4 * Math.PI);
        
    }

    private double simDrivePosition;

    @SuppressWarnings("rawtypes")
    public double getDrivePosition(){
        if(RobotBase.isReal()){
            StatusSignal pos = drive.getPosition();
            return pos.getValueAsDouble() * (4 * Math.PI);
        }
        //IF in sim

        //TODO this will only ever work with a 7 ms loop
        simDrivePosition += getDriveVelocity() * 0.07 /10;//added fudge factor so it is somewhat accurate to real life
        return simDrivePosition;
    }
}