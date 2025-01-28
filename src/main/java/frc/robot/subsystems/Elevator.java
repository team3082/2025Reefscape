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
    
    public TalonFX motor1, motor2;

    private double targetHeight;

    public Elevator() {
        motor1 = new TalonFX(Constants.Elevator.MOTORID1, "CANivore");
        motor2 = new TalonFX(Constants.Elevator.MOTORID2, "CANivore");

        motor1.getConfigurator().apply(new TalonFXConfiguration());
        motor2.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration motor1Config = new TalonFXConfiguration();
        motor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor1Config.Slot0.kP = Tuning.Elevator.ELEVATOR_P;
        motor1Config.Slot0.kI = Tuning.Elevator.ELEVATOR_I;
        motor1Config.Slot0.kD = Tuning.Elevator.ELEVATOR_D;

        motor1Config.MotionMagic.MotionMagicCruiseVelocity = Tuning.Elevator.MOTION_MAGIC_CRUISE_VELOCITY;
        motor1Config.MotionMagic.MotionMagicAcceleration = Tuning.Elevator.MOTION_MAGIC_ACCELERATION;
        motor1Config.MotionMagic.MotionMagicJerk = Tuning.Elevator.JERK;

        TalonFXConfiguration motor2Config = new TalonFXConfiguration();

        motor1.getConfigurator().apply(motor1Config);
        motor2.getConfigurator().apply(motor2Config);

        Follower follower = new Follower(Constants.Elevator.MOTORID1, true);

        motor2.setControl(follower);
    }

    public void update() {
        motor1.setControl(new MotionMagicDutyCycle(inchToRot(targetHeight)));

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

    public double getElevatorHeight() {
        if (Robot.isReal()) return rotToInch(motor1.getPosition().getValueAsDouble());
        else return ElevatorSim.getPosition();
    }

    public boolean atPosition() {
        return Math.abs(getElevatorHeight() - targetHeight) < Tuning.Elevator.HEIGHT_DEADBAND;
    }

    private double inchToRot(double inch) {
        return inch / Constants.Elevator.INCHESPERROTATION;
    }

    private double rotToInch(double rot) {
        return rot * Constants.Elevator.INCHESPERROTATION;
    }

}
