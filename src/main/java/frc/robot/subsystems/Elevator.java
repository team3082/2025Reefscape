package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.subsystems.sim.ElevatorSim;

public class Elevator {
    // hardware
    public TalonFX extensionMotor1; 
    public TalonFX extensionMotor2;

    // state
    public double targetHeight;

    /** Constructor */
    public Elevator() {
        init();
    }

    /** Initialize Elevator Subsystem - only call in constructor */
    public void init() {
        // Initialize Motors
        extensionMotor1 = new TalonFX(Constants.Elevator.MOTORID1, "CANivore");
        extensionMotor2 = new TalonFX(Constants.Elevator.MOTORID2, "CANivore");

        extensionMotor1.getConfigurator().apply(new TalonFXConfiguration());
        extensionMotor2.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration extensionMotor1Config = new TalonFXConfiguration();
        extensionMotor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        extensionMotor1Config.Slot0.kP = Tuning.Elevator.ELEVATOR_P;
        extensionMotor1Config.Slot0.kI = Tuning.Elevator.ELEVATOR_I;
        extensionMotor1Config.Slot0.kD = Tuning.Elevator.ELEVATOR_D;

        extensionMotor1Config.MotionMagic.MotionMagicCruiseVelocity = Tuning.Elevator.MOTION_MAGIC_CRUISE_VELOCITY;
        extensionMotor1Config.MotionMagic.MotionMagicAcceleration = Tuning.Elevator.MOTION_MAGIC_ACCELERATION;
        extensionMotor1Config.MotionMagic.MotionMagicJerk = Tuning.Elevator.JERK;

        TalonFXConfiguration extensionMotor2Config = new TalonFXConfiguration();

        extensionMotor1.getConfigurator().apply(extensionMotor1Config);
        extensionMotor2.getConfigurator().apply(extensionMotor2Config);

        // set second motor to follow master motor
        Follower follower = new Follower(Constants.Elevator.MOTORID1, true);
        extensionMotor2.setControl(follower);
    }
    /** applies elevator positional control,
     *  only call in ScoringManager.update()
    */
    public void update() {
        extensionMotor1.setControl(new MotionMagicDutyCycle(inchToRot(targetHeight)));

        // UPDATE SIM
        if (Robot.isSimulation()) {
            ElevatorSim.setPosition(targetHeight);
            ElevatorSim.update();
        }
    }

    /**
     * sets the height of the elevator
     * @param targetHeight target height in inches
     */
    public void setElevatorHeight(double targetHeight) {
        this.targetHeight = targetHeight;
    }

    /** get the elevator height in inches */
    public double getElevatorHeight() {
        if (Robot.isReal()) return rotToInch(extensionMotor1.getPosition().getValueAsDouble());
        else return ElevatorSim.getPosition();
    }

    /** returns true if elevator position is within set deadband */
    public boolean atPosition() {
        return Math.abs(getElevatorHeight() - targetHeight) < Tuning.Elevator.HEIGHT_DEADBAND;
    }

    /** converts inches to internal motor rotations */
    private double inchToRot(double inch) {
        return inch / Constants.Elevator.INCHESPERROTATION;
    }

    /** converts internal motor rotations to inches */
    private double rotToInch(double rot) {
        return rot * Constants.Elevator.INCHESPERROTATION;
    }

}
