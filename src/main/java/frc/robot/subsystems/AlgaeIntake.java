package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Tuning;

public class AlgaeIntake {

    private static TalonFX pivotMotor;
    private static TalonFX topWheelMotor;
    public static DigitalInput sensor;

    public static IntakeState state = IntakeState.STOW;
    
    public enum IntakeState {
        STOW(Tuning.Intake.STOW_ANGLE, 0.0),
        FEED(Tuning.Intake.FEED_ANGLE, Tuning.Intake.FEED_SPEED),
        EJECT(Tuning.Intake.EJECT_ANGLE, Tuning.Intake.EJECT_SPEED),
        HOLD(Tuning.Intake.HOLD_ANGLE, 0.0),
        DISABLED(0.0, 0.0);

        public double targetAngle;
        public double targetSpeed;

        IntakeState(double targetAngle, double targetSpeed) {
            this.targetAngle = targetAngle;
            this.targetSpeed = targetSpeed;
        }
    }

    public static void init() {

        pivotMotor = new TalonFX(Constants.AlgaeIntake.INTAKEPIVOT_ID, "CANivore");
        topWheelMotor = new TalonFX(Constants.AlgaeIntake.TOP_MOTOR_ID, "CANivore");

        // Pivot motor config
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        
        pivotConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;
        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        pivotConfig.Slot0.kP = 0.0;
        pivotConfig.Slot0.kI = 0.0;
        pivotConfig.Slot0.kD = 0.0;

        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 40000;
        pivotConfig.MotionMagic.MotionMagicAcceleration = 40000;

        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Top Wheel motor config
        TalonFXConfiguration topWheelConfig = new TalonFXConfiguration();

        topWheelConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;
        topWheelConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        topWheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Apply configs
        pivotMotor.getConfigurator().apply(pivotConfig);
        topWheelMotor.getConfigurator().apply(topWheelConfig);

        // Beambreak
        sensor = new DigitalInput(Constants.AlgaeIntake.SENSOR_CHANNEL);
    }

    public static void update() {
        switch (state) {
            case DISABLED:
                pivotMotor.setNeutralMode(NeutralModeValue.Coast);
                topWheelMotor.setNeutralMode(NeutralModeValue.Coast);

            break;

            default:
                pivotMotor.setPosition(radToRot(state.targetAngle));
                topWheelMotor.set(state.targetSpeed);

            break;
        }
    }

    public static void setState(IntakeState newState) {
        state = newState;
    }

    // Convert radians to motor rotations
    public static double radToRot(double radians) {
        return radians / (2.0 * Math.PI) * Constants.AlgaeIntake.INTAKE_GEAR_RATIO;
    }
}
