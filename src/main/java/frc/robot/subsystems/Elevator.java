package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.subsystems.sim.ElevatorSim;

public class Elevator {
    public TalonFX extensionMotor1; 
    public TalonFX extensionMotor2;

    public double targetHeight;

    public Elevator() {
        init();
    }

    /** only call in constructor */
    public void init() {
        extensionMotor1 = new TalonFX(Constants.Elevator.MOTORID1, "CANivore");
        extensionMotor2 = new TalonFX(Constants.Elevator.MOTORID2, "CANivore");

        extensionMotor1.getConfigurator().apply(new TalonFXConfiguration());
        extensionMotor2.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration extensionMotor1Config = new TalonFXConfiguration();
        extensionMotor1Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        extensionMotor1Config.Slot0.kP = Tuning.Elevator.ELEVATOR_P;
        extensionMotor1Config.Slot0.kI = Tuning.Elevator.ELEVATOR_I;
        extensionMotor1Config.Slot0.kD = Tuning.Elevator.ELEVATOR_D;

        extensionMotor1Config.MotionMagic.MotionMagicCruiseVelocity = Tuning.Elevator.MOTION_MAGIC_CRUISE_VELOCITY;
        extensionMotor1Config.MotionMagic.MotionMagicAcceleration = Tuning.Elevator.MOTION_MAGIC_ACCELERATION;
        extensionMotor1Config.MotionMagic.MotionMagicJerk = Tuning.Elevator.JERK;

        TalonFXConfiguration extensionMotor2Config = new TalonFXConfiguration();

        extensionMotor2Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        extensionMotor1.getConfigurator().apply(extensionMotor1Config);
        extensionMotor2.getConfigurator().apply(extensionMotor2Config);

        // set second motor to follow master motor
        Follower follower = new Follower(Constants.Elevator.MOTORID1, true);
        extensionMotor2.setControl(follower);

        extensionMotor1.setPosition(0);
        extensionMotor2.setPosition(0);
    }

    /** applies elevator positional control,
     *  only call in ScoringManager.update()
    */
    public void update() {
        extensionMotor1.setControl(new PositionDutyCycle(inchToRot(targetHeight)));

        // UPDATE SIM
        if (Robot.isSimulation()) {
            ElevatorSim.setPosition(targetHeight);
            ElevatorSim.update();
        }
    }

    /**
     * @param targetHeight target height in inches
     */
    public void setElevatorHeight(double targetHeight) {
        this.targetHeight = targetHeight;
    }

    /** @return height in inches */
    public double getElevatorHeight() {
        if (Robot.isReal()) return rotToInch(extensionMotor1.getPosition().getValueAsDouble());
        else return ElevatorSim.getPosition();
    }

    /** @return true if elevator position is within set deadband */
    public boolean atPosition() {
        return Math.abs(getElevatorHeight() - targetHeight) < Tuning.Elevator.HEIGHT_DEADBAND;
    }

    /** converts inches to internal motor rotations */
    private double inchToRot(double inch) {
        return inch; // not needed currenly
    }

    /** converts internal motor rotations to inches */
    private double rotToInch(double rot) {
        return rot; // not needed currently
    }

    public void disable(){
        extensionMotor1.setControl(new StaticBrake());
    }

    public void test() {
        extensionMotor1.setControl(new NeutralOut());
    }

}
