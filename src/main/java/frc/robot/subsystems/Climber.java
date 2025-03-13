package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.subsystems.sim.ClimberSim;

/**
 * Represents the climber subsystem and has methods for climbing
 */
public class Climber {
    private static ClimbState climbState;
    public static TalonFX rotationMotor;

    public enum ClimbState {
        RESTING(Constants.Climber.RESTINGANGLE),
        CLIMBING(Constants.Climber.CLIMBINGANGLE);

        public double targetRadians;

        ClimbState(double targetRadians) {
            this.targetRadians = targetRadians;
        }
    }

    /**
     * Inits the Climber Subystem
     */
    public static void init() {
        rotationMotor = new TalonFX(Constants.Climber.ROTMOTORID);
        climbState = ClimbState.RESTING;

        rotationMotor.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration rotConfig = new TalonFXConfiguration();

        rotConfig.Slot0.kP = Tuning.Climber.CLIMBER_P;
        rotConfig.Slot0.kI = Tuning.Climber.CLIMBER_I;
        rotConfig.Slot0.kD = Tuning.Climber.CLIMBER_D;

        rotationMotor.getConfigurator().apply(rotConfig);
    }

    /**
     * Updates the Climber Subsytem
     */
    public static void update() {
        if (Robot.isReal()) {
            double raidansToRotations = 1/(Math.PI * 2 * Constants.Climber.GEARRATIO);
            rotationMotor.setPosition(climbState.targetRadians * raidansToRotations);
        }
        
        if (Robot.isSimulation()) {
            ClimberSim.setAngle(climbState.targetRadians);
            ClimberSim.update();
        }
    }

    /**
     * Sets the Climber State
     * @param newState The new state
     */
    public static void setState(ClimbState newState) {
        climbState = newState;
    }

    /**
     * Gets the climber state
     * @return A Climbstate Enum
     */
    public static ClimbState getState() {
        return climbState;
    }

    /**
     * Gets the climber angle
     * @return A double in radians
     */
    public static double getClimberAngle() {
        if (Robot.isReal()) {
            return rotationMotor.getPosition().getValueAsDouble() * Math.PI * 2 * Constants.Climber.GEARRATIO;
        }

        return ClimberSim.getAngle();
    }
}
