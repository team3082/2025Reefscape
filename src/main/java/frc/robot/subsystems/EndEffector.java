package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.subsystems.sim.EndEffectorSim;
import frc.robot.utils.RTime;

public class EndEffector {

    // contains all intake wheel control states, stores value for set wheel speed
    public enum IntakeState {
        OFF(0.0),
        INTAKE_CORAL(-0.285),
        HOLD_CORAL(0.0),
        DROP_CORAL(-0.5),
        INTAKE_ALGAE(0.5),
        DROP_ALGAE(-0.5),
        L1(0.275),
        STOW(-0.2),
        L4(-0.375);

        public double targetSpeed;

        IntakeState(double targetSpeed) {
            this.targetSpeed = targetSpeed;
        }
    }

    // hardware
    public TalonFX pivotMotor;
    public TalonFX intakeMotor;
    public DigitalInput sensor;

    // state
    public IntakeState intakeState = IntakeState.HOLD_CORAL;
    public double targetAngle; // Radians
    public boolean holdingPiece;

    public EndEffector() {
        init();
    }

    /** Initialize End Effector Subsystem - only call in constructor */
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
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = Tuning.EndEffector.MOTION_MAGIC_CRUISE_VELOCITY;
        pivotConfig.MotionMagic.MotionMagicAcceleration = Tuning.EndEffector.MOTION_MAGIC_ACCELERATION;

        // Apply Configs
        pivotMotor.getConfigurator().apply(pivotConfig);

        // Initialize Intake Motor
        intakeMotor = new TalonFX(Constants.EndEffector.INTAKEID, "CANivore");

        // Reset to Factory Defaults
        intakeMotor.getConfigurator().apply(new TalonFXConfiguration());

        // Initialize Sensor
        sensor = new DigitalInput(Constants.EndEffector.END_EFFECTOR_SENSOR_ID);

        pivotMotor.setPosition(0.0);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    /** applies intake motor speeds based on intake state,
     *  applies wrist motor positional control,
     *  only call in ScoringManager.update()
    */
    public void update() {
        holdingPiece = isHoldingCoral();

        switch (intakeState) {
            case INTAKE_CORAL:
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

    /** @return the pivot angle in radians */
    public double getPivotAngle() {
        if (Robot.isReal()) return rotToRad(pivotMotor.getPosition().getValueAsDouble());
        else return EndEffectorSim.getPosition();
    }

    /** @return if current motor position is within a set deadband */
    public boolean atPosition() {
        return Math.abs(getPivotAngle() - targetAngle) <= Tuning.EndEffector.PIVOT_DEADBAND;
    }

    /** conversion from radians to internal motor rotations */
    private double radToRot(double rad) {
        return (rad / (2.0 * Math.PI)) * Constants.EndEffector.GEARRATIO;
    }   

    /** conversion from internal motor rotations to radians */
    private double rotToRad(double rot) {
        return (rot * 2.0 * Math.PI) / Constants.EndEffector.GEARRATIO;
    }

    public void intake() {
        switch (ScoringManager.scoringPosition) {
            case ALGAE_INTAKE:
                setIntakeState(IntakeState.INTAKE_ALGAE);
                break;
            default:
                setIntakeState(IntakeState.INTAKE_CORAL);
                break;
        }
    }

    public void outtake() {
        switch (ScoringManager.scoringPosition) {
            case ALGAE_INTAKE:
                setIntakeState(IntakeState.DROP_ALGAE);
                break;
            case STOW:
                setIntakeState(IntakeState.STOW);
                break;
            case L1:
                setIntakeState(IntakeState.L1);
                break;
            case L4:
                setIntakeState(IntakeState.L4);
                break;
            default:
                setIntakeState(IntakeState.DROP_CORAL);
                break;
        }
    }

    public boolean beambreakBroken = false;
    public double suckTime = 0.0;
    public double neededSuckTime = 0.00; // the delay to stop moving the coral at the right spot

    /**
     * Gets if the end effector has held the piece for the necessary amount of time
     * @return if the end effector is holding coral in the right spot
     */
    public boolean isHoldingCoral() {
        if (!sensor.get()) {
            if (!beambreakBroken) {
                suckTime = RTime.now();
            }
            beambreakBroken = true;
            if (RTime.now() >= suckTime + neededSuckTime) {
                return true;
            } else {
                return false;
            }
        } else {
            beambreakBroken = false;
            return false;
        }
    }

    public void disable() {
        pivotMotor.setControl(new StaticBrake());
    }

    public void test() {
        pivotMotor.setControl(new NeutralOut());
    }

}
