package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.subsystems.sim.EndEffectorSim;

public class EndEffector {

    public enum IntakeState {
        OFF(0.0),
        INTAKE_PIECE(-0.15),
        HOLD_PIECE(0.0),
        DROP_PIECE(-0.5);

        public double targetSpeed;

        IntakeState(double targetSpeed) {
            this.targetSpeed = targetSpeed;
        }
    }

    public TalonFX pivotMotor;
    public TalonFX intakeMotor;
    public DigitalInput sensor;

    public IntakeState intakeState = IntakeState.HOLD_PIECE;

    public double targetAngle; // Radians
    public boolean holdingPiece;

    public EndEffector() {
        init();
    }

    public void init() {
        // Initialize Motors
        pivotMotor = new TalonFX(Constants.EndEffector.PIVOTID, "CANivore");

        // Reset to Factory Defaults
        pivotMotor.getConfigurator().apply(new TalonFXConfiguration());

        // Set Pivot Motor Configs
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        // PID Configs
        pivotConfig.Slot0.kP = Tuning.EndEffector.PIVOT_P;
        pivotConfig.Slot0.kI = Tuning.EndEffector.PIVOT_I;
        pivotConfig.Slot0.kD = Tuning.EndEffector.PIVOT_D;

        // Motion Magic Configs
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = Tuning.EndEffector.MOTION_MAGIC_CRUISE_VELOCITY;
        pivotConfig.MotionMagic.MotionMagicAcceleration = Tuning.EndEffector.MOTION_MAGIC_ACCELERATION;
        pivotConfig.MotionMagic.MotionMagicJerk = Tuning.EndEffector.JERK;

        // Apply Configs
        pivotMotor.getConfigurator().apply(pivotConfig);

        // Initialize Intake Motor
        intakeMotor = new TalonFX(Constants.EndEffector.PIVOTID, "CANivore");

        // Reset to Factory Defaults
        intakeMotor.getConfigurator().apply(new TalonFXConfiguration());

        // Initialize Sensor
        sensor = new DigitalInput(Constants.EndEffector.END_EFFECTOR_SENSOR_ID);
    }

    public void update() {
        holdingPiece = !sensor.get();

        switch (intakeState) {
            case INTAKE_PIECE:
                // Stop Wheels if Holding Piece
                if (holdingPiece) intakeMotor.set(0);
                else intakeMotor.set(intakeState.targetSpeed);
            break;
            default:
                intakeMotor.set(intakeState.targetSpeed);
            break;
        }

        // setting pivot angle
        pivotMotor.setControl(new MotionMagicDutyCycle(radToRot(targetAngle)));

        // UPDATE SIM
        if (Robot.isSimulation()) {
            EndEffectorSim.setPosition(targetAngle);
            EndEffectorSim.setSpeed(intakeState.targetSpeed);
            EndEffectorSim.update();
        }
    }

    public void setIntakeState(IntakeState newState) {
        intakeState = newState;
    }

    public void setPivotAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public double getPivotAngle() {
        if (Robot.isReal()) return rotToRad(pivotMotor.getPosition().getValueAsDouble());
        else return EndEffectorSim.getPosition();
    }

    public boolean atPosition() {
        return Math.abs(getPivotAngle() - targetAngle) <= Tuning.EndEffector.PIVOT_DEADBAND;
    }

    private double radToRot(double rad) {
        return rad / (2.0 * Math.PI) * Constants.EndEffector.GEARRATIO;
    }   

    private double rotToRad(double rot) {
        return (rot * 2.0 * Math.PI) / Constants.EndEffector.GEARRATIO;
    }

}
