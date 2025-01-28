package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Tuning;

public class EndEffector {

    public enum IntakeState {
        OFF,
        INTAKE_PIECE,
        HOLD_PIECE,
        DROP_PIECE,
    }

    public static TalonFX pivot;
    public static TalonFX intake;

    public static IntakeState intakeState;

    public static DigitalInput sensor;

    public static boolean holdingPiece;
    
    public static void init() {

        pivot = new TalonFX(Constants.EndEffector.PIVOTID, "CANivore");
        pivot.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();
        pivotConfiguration.Slot0.kP = Tuning.EndEffector.PIVOT_P;
        pivotConfiguration.Slot0.kI = Tuning.EndEffector.PIVOT_I;
        pivotConfiguration.Slot0.kD = Tuning.EndEffector.PIVOT_D;

        pivotConfiguration.MotionMagic.MotionMagicCruiseVelocity = Tuning.EndEffector.MOTION_MAGIC_CRUISE_VELOCITY;
        pivotConfiguration.MotionMagic.MotionMagicAcceleration = Tuning.EndEffector.MOTION_MAGIC_ACCELERATION;
        pivotConfiguration.MotionMagic.MotionMagicJerk = Tuning.EndEffector.JERK;

        pivot.getConfigurator().apply(pivotConfiguration);

        intake = new TalonFX(Constants.EndEffector.PIVOTID, "CANivore");
        intake.getConfigurator().apply(new TalonFXConfiguration());

        sensor = new DigitalInput(Constants.EndEffector.END_EFFECTOR_SENSOR_ID);

        intakeState = IntakeState.HOLD_PIECE;

    }

    public static void update() {

        holdingPiece = sensor.get();

        switch (intakeState) {
            case OFF:

                intake.setControl(new DutyCycleOut(0));
                
                break;

            case INTAKE_PIECE:

                if (holdingPiece) {
                    intakeState = IntakeState.HOLD_PIECE;
                }

                intake.setControl(new DutyCycleOut(Tuning.EndEffector.INTAKE_SPEED));

                break;

            case HOLD_PIECE:

                intake.setControl(new DutyCycleOut(0));

                break;

            case DROP_PIECE:

                intake.setControl(new DutyCycleOut(Tuning.EndEffector.INTAKE_SPEED));

                break;

        }
    }

    public static void setIntakeState(IntakeState newState) {
        intakeState = newState;
    }

    public static void setPivotAngle(double targetAngle) {
        pivot.setControl(new MotionMagicDutyCycle(targetAngle / (2.0 * Math.PI) * Constants.EndEffector.GEARRATIO));
    }

    public static double getPivotAngle() {
        return pivot.getPosition().getValueAsDouble() / Constants.EndEffector.GEARRATIO;
    }

    public static boolean atPosition(double targetAngle) {
        return Math.abs(EndEffector.getPivotAngle() - targetAngle) <= Tuning.EndEffector.PIVOT_DEADBAND;
    }


}
