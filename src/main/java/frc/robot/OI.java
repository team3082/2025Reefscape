package frc.robot;

import static frc.robot.Tuning.OI.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.controllermaps.LogitechF310;
import frc.robot.subsystems.ScoringManager;
import frc.robot.subsystems.EndEffector.IntakeState;
import frc.robot.subsystems.ScoringManager.ScoringPosition;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePID;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.Vector2;
import frc.robot.utils.RMath;

public class OI {
    public static Joystick driverStick;

    // Driver Controls

    // Movement
    static final int moveX         = LogitechF310.AXIS_LEFT_X;
    static final int moveY         = LogitechF310.AXIS_LEFT_Y;
    // static final int rotateX       = LogitechF310.AXIS_RIGHT_X;
    static final int rotateX       = 2;
    static final int boost         = LogitechF310.AXIS_RIGHT_TRIGGER;

    // zero is for Pigeon
    static final int zero          = LogitechF310.BUTTON_Y;

    static final int funnyButton   = LogitechF310.BUTTON_A;
    private static boolean drivingToReef = false;

    // Operator Controls
    public static Joystick operatorStick;

    // Operator Controls

    // Scoring Positions
    static final int stow        = LogitechF310.BUTTON_A;
    static final int L2          = LogitechF310.BUTTON_B;
    static final int L3          = LogitechF310.BUTTON_X;
    static final int L4          = LogitechF310.BUTTON_Y;
    
    // End Effector Control
    static final int intake       = LogitechF310.BUTTON_LEFT_BUMPER;
    static final int outtake      = LogitechF310.BUTTON_RIGHT_BUMPER;

    /**
     * Initialize OI with preset joystick ports.
     */
    public static void init() {
        driverStick = new Joystick(0);
        operatorStick = new Joystick(1); // Temporarily port 0 for sim testing
    }

    public static void userInput() {
        driverInput();
        operatorInput();
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
        // System.out.println("drive direction: " + drive.atan2());
        System.out.println("drive magnitude: " + drive.mag());
        if (Robot.isSimulation() && drive.mag() > 0.4) {
            drive = drive.norm(); 
            drive = drive.mul(0.4);
        }
        // SCORING
        if (driverStick.getRawButtonPressed(funnyButton)) {
            drivingToReef = !drivingToReef;
            if(drivingToReef) {
                Vector2 currentPos = SwervePosition.getPosition();
                int allianceStartIndex = 6;
                // Determine reef AprilTag locations based on alliance
                if(Robot.isReal())
                    allianceStartIndex = DriverStation.getAlliance().get() == Alliance.Red ? 6 : 17;

                // Find the shortest scoring position from the robot
                double min = currentPos.sub(Constants.APRIL_TAGS[allianceStartIndex].getPosition()).mag();
                int minIndex = allianceStartIndex;
                for (int i = allianceStartIndex+1; i < allianceStartIndex + 6; i++){
                    System.out.println(Constants.APRIL_TAGS[i].getPosition());
                    Vector2 aprilPosition = Constants.APRIL_TAGS[i].getPosition();
                    if(currentPos.sub(aprilPosition).mag() < min){
                        minIndex = i;
                        min = currentPos.sub(aprilPosition).mag();
                    }
                }
                
                // TODO Adjust positions for accurate scoring
                // Set destination and rotation based on AprilTag data
                SwervePID.setDestPt(Constants.APRIL_TAGS[minIndex].getPosition());
                SwervePID.setDestRot(Constants.APRIL_TAGS[minIndex].getRotationZ() + (Math.PI));
            }
        }

        /*--------------------------------------------------------------------------------------------------------*/
        // SWERVE
        if (drivingToReef){
            if(SwervePID.atDest() &&  SwervePID.atRot()){
                drivingToReef = !drivingToReef;
            }
            SwerveManager.rotateAndDrive(SwervePID.updateOutputRot(), SwervePID.updateOutputVel());
        } else {
            SwerveManager.rotateAndDrive(rotate, drive);
        }
    }

    public static void operatorInput() {
        /*-Scoring Manager----------------------------------------------------------------------------------------*/
        if (operatorStick.getRawButtonPressed(stow)) ScoringManager.setScoringLevel(ScoringPosition.STOW);
        else if (operatorStick.getRawButtonPressed(L2)) ScoringManager.setScoringLevel(ScoringPosition.L2);
        else if (operatorStick.getRawButtonPressed(L3)) ScoringManager.setScoringLevel(ScoringPosition.L3);
        else if (operatorStick.getRawButtonPressed(L4)) ScoringManager.setScoringLevel(ScoringPosition.L4);

        /*-End Effector-------------------------------------------------------------------------------------------*/
        if (operatorStick.getRawButton(intake)) ScoringManager.endEffector.setIntakeState(IntakeState.INTAKE_PIECE);
        else if (operatorStick.getRawButton(outtake)) ScoringManager.endEffector.setIntakeState(IntakeState.DROP_PIECE);
        else ScoringManager.endEffector.setIntakeState(IntakeState.HOLD_PIECE);
    }

}
