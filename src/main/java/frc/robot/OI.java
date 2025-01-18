package frc.robot;

import static frc.robot.Tuning.OI.*;

import edu.wpi.first.wpilibj.Joystick;
import frc.controllermaps.LogitechF310;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwerveManager;
import frc.robot.utils.Vector2;
import frc.robot.utils.RMath;

public class OI {
    public static Joystick driverStick;

    // Driver Controls

    // Movement
    static final int moveX         = LogitechF310.AXIS_LEFT_X;
    static final int moveY         = LogitechF310.AXIS_LEFT_Y;
    static final int rotateX       = LogitechF310.AXIS_RIGHT_X;
    static final int boost         = LogitechF310.AXIS_RIGHT_TRIGGER;

    // zero is for Pigeon
    static final int zero          = LogitechF310.BUTTON_Y;

    /**
     * Initialize OI with preset joystick ports.
     */
    public static void init() {
        driverStick = new Joystick(0);
    }

    public static void userInput() {
        driverInput();
    }

    /**
     * Instruct the robot to follow instructions from joysticks.
     * One call from this equals one frame of robot instruction.
     * Because we used TimedRobot, this runs 50 times a second,
     * so this lives in the teleopPeriodic() function.
     */
    public static void driverInput() {
        // INPUT

        if (driverStick.getRawButton(zero)) Pigeon.zero();

        double boostStrength = driverStick.getRawAxis(boost);
        if(boostStrength < 0.1) boostStrength = 0;

        double kBoostCoefficient = NORMALSPEED + boostStrength * (1.0 - NORMALSPEED);

        /*--------------------------------------------------------------------------------------------------------*/
        // SETUP

        Vector2 drive = new Vector2(driverStick.getRawAxis(moveX), -driverStick.getRawAxis(moveY));
        double rotate = RMath.smoothJoystick1(driverStick.getRawAxis(rotateX)) * -ROTSPEED;

        if (drive.mag() < 0.125) {
            drive = new Vector2();
        } else {
            drive = RMath.smoothJoystick2(drive).mul(kBoostCoefficient);
        }
        if (Math.abs(rotate) < 0.005) {
            rotate = 0;
        }
        /*--------------------------------------------------------------------------------------------------------*/
        // SWERVE

        SwerveManager.rotateAndDrive(rotate, drive);
        
    }


}