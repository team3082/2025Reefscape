package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.Tuning;

public class EndEffector {

    public static TalonFX pivot;
    
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
