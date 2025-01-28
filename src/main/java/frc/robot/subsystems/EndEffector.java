package frc.robot.subsystems;

import javax.xml.stream.events.EndDocument;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
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
        INTAKE_PIECE(0.15),
        HOLD_PIECE(0.0),
        DROP_PIECE(-0.5);

        public double speed;

        IntakeState(double speed) {
            this.speed = speed;
        }
    }

    public TalonFX pivot;
    public TalonFX intake;

    private double targetAngle; // Radians

    public IntakeState intakeState;

    public DigitalInput sensor;

    public boolean holdingPiece;

    public EndEffector() {

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

    public void update() {
        holdingPiece = !sensor.get();

        switch (intakeState) {
            case INTAKE_PIECE:
                // Stop Wheels if Holding Piece
                if (holdingPiece) intake.set(0);
                else intake.set(intakeState.speed);
            break;
            default:
                intake.set(intakeState.speed);
            break;
        }

        // setting pivot angle
        pivot.setControl(new MotionMagicDutyCycle(radToRot(targetAngle)));

        // UPDATE SIM
        if (Robot.isSimulation()) {
            EndEffectorSim.setPosition(targetAngle);
            EndEffectorSim.setSpeed(intakeState.speed);
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
        if (Robot.isReal()) return rotToRad(pivot.getPosition().getValueAsDouble());
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
