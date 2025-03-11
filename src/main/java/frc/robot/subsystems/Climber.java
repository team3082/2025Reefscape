package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.subsystems.sim.ClimberSim;

public class Climber {
    // gear ratio 115:1
    private static ClimbState state;

    public static TalonFX rotationMotor;

    public enum ClimbState {
        RESTING(Constants.Climber.RESTINGANGLE),
        CLIMBING(Constants.Climber.CLIMBINGANGLE);

        public double targetAngle;

        ClimbState(double targetAngle) {
            this.targetAngle = targetAngle;
        }
    }

    public static void init() {
        rotationMotor = new TalonFX(Constants.Climber.ROTMOTORID);
        state = ClimbState.RESTING;
        rotationMotor.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration rotConfig = new TalonFXConfiguration();

        // PID Configs
        rotConfig.Slot0.kP = Tuning.Climber.CLIMBER_P;
        rotConfig.Slot0.kI = Tuning.Climber.CLIMBER_I;
        rotConfig.Slot0.kD = Tuning.Climber.CLIMBER_D;

        // Apply Configs
        rotationMotor.getConfigurator().apply(rotConfig);
    }

    public static void update() {
        if (Robot.isReal()) {
            rotationMotor.setPosition(state.targetAngle);
        }
        
        if (Robot.isSimulation()) {
            ClimberSim.setAngle(state.targetAngle);
            ClimberSim.update();
        }
    }

    public static void setState(ClimbState newState) {
        state = newState;
    }

    public static ClimbState getState() {
        return state;
    }

    public static double getClimberAngle() {
        if (Robot.isReal()) {
            return rotationMotor.getPosition().getValueAsDouble() * Math.PI * 2 * Constants.Climber.GEARRATIO;
        } else {
            return ClimberSim.getAngle();
        }
    }

}
