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
import frc.robot.subsystems.visualizer.CoralVisualizer;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePID;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.Vector2;
import frc.robot.utils.RMath;

public class OI {
    public static Joystick driverStick;

    // ------------------ Driver Controls ------------------ //

    // Movement
    static final int moveX                       = LogitechF310.AXIS_LEFT_X;
    static final int moveY                       = LogitechF310.AXIS_LEFT_Y;
    static final int rotateX                     = LogitechF310.AXIS_RIGHT_X;
    
    static final int boost                       = LogitechF310.AXIS_RIGHT_TRIGGER;
 
    // zero is for Pigeon 
    static final int zero                        = LogitechF310.BUTTON_Y;
 
    static final int funnyButtonLeft             = LogitechF310.DPAD_LEFT;
    static final int funnyButtonRight            = LogitechF310.DPAD_RIGHT;
 
    private static final int lockIn              = LogitechF310.BUTTON_X;
    private static final int lockOut             = LogitechF310.BUTTON_B;
 
    // End Effector Control 
    static final int intake                      = LogitechF310.BUTTON_LEFT_BUMPER;
    static final int outtake                     = LogitechF310.BUTTON_RIGHT_BUMPER;
    static final int dealgaefy                   = LogitechF310.AXIS_LEFT_TRIGGER;
 
    private static boolean drivingToReef         = false;
    private static boolean previouslyPressedPOV  = false; // Checks if we previously pressed the dpad because getPOV() doesn't do that
 
 
    // ------------------ Operator Controls ------------------ //
    public static Joystick operatorStick; 
 
    // Scoring Positions 
    private static final int stow                = LogitechF310.BUTTON_A;
    private static final int L2                  = LogitechF310.BUTTON_B;
    private static final int L3                  = LogitechF310.BUTTON_X;
    private static final int L4                  = LogitechF310.BUTTON_Y;
    private static final int ALGAE1              = LogitechF310.DPAD_DOWN;
    private static final int ALGAE2              = LogitechF310.DPAD_UP;
    private static final int LET_HIM_COOK        = LogitechF310.DPAD_LEFT;
 
    private static final int RightTrigger        = LogitechF310.AXIS_RIGHT_TRIGGER;
    private static final int LeftTrigger         = LogitechF310.AXIS_LEFT_TRIGGER;


    private static ScoringPosition savedLevel = ScoringPosition.STOW;

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

        // Reset pigeon
        if (driverStick.getRawButton(zero)) Pigeon.reset();

        double boostStrength = driverStick.getRawAxis(boost);
        if(boostStrength < 0.1) boostStrength = 0;

        double kBoostCoefficient = NORMALSPEED + boostStrength * (1.0 - NORMALSPEED);

        /*--------------------------------------------------------------------------------------------------------*/
        // SETUP

        Vector2 drive = new Vector2(driverStick.getRawAxis(moveX), -driverStick.getRawAxis(moveY));
        double rotate =  RMath.smoothJoystick1(driverStick.getRawAxis(rotateX)) * -ROTSPEED;

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

        // SCORING
        if ((driverStick.getPOV() == funnyButtonLeft || driverStick.getPOV() == funnyButtonRight) && !previouslyPressedPOV) {
            previouslyPressedPOV = true;
            drivingToReef = !drivingToReef;
            if(drivingToReef) {
                Vector2 currentPos = SwervePosition.getPosition();
                int allianceStartIndex = 6;
                // Determine reef AprilTag locations based on alliance
                allianceStartIndex = DriverStation.getAlliance().get() == Alliance.Red ? 6 : 17;

                // Find the shortest scoring position from the robot
                double min;
                if (driverStick.getPOV() == funnyButtonLeft)
                    min = currentPos.sub(Constants.APRIL_TAGS[allianceStartIndex].getLeftPosition()).mag();
                else 
                    min = currentPos.sub(Constants.APRIL_TAGS[allianceStartIndex].getRightPosition()).mag();
                
                int minIndex = allianceStartIndex;
                for (int i = allianceStartIndex+1; i < allianceStartIndex + 6; i++){
                    System.out.println(Constants.APRIL_TAGS[i].getPosition());
                    Vector2 aprilPosition;
                    if (driverStick.getPOV() == funnyButtonLeft)
                        aprilPosition = Constants.APRIL_TAGS[i].getLeftPosition();
                    else 
                        aprilPosition = Constants.APRIL_TAGS[i].getRightPosition();

                    if(currentPos.sub(aprilPosition).mag() < min){
                        minIndex = i;
                        min = currentPos.sub(aprilPosition).mag();
                    }
                }
                
                // TODO Adjust positions for accurate scoring
                // Set destination and rotation based on AprilTag data
                Vector2 targetPosition = driverStick.getPOV() == funnyButtonRight ? Constants.APRIL_TAGS[minIndex].getLeftPosition() : Constants.APRIL_TAGS[minIndex].getRightPosition();
                SwervePID.setDestPt(targetPosition);
                
                SwervePID.setDestRot(Constants.APRIL_TAGS[minIndex].getRotationZ() + (DriverStation.getAlliance().get() == Alliance.Blue ? -1 : 1) * Math.PI/2);

            }
        } else {
            previouslyPressedPOV = false;
        }

        /*-End Effector-------------------------------------------------------------------------------------------*/
        if (driverStick.getRawButton(intake)) ScoringManager.endEffector.setIntakeState(IntakeState.INTAKE_PIECE);
        else if (driverStick.getRawButton(outtake)) ScoringManager.endEffector.setIntakeState(IntakeState.DROP_PIECE);
        else if (driverStick.getRawAxis(dealgaefy) > 0.7)  ScoringManager.endEffector.setIntakeState(IntakeState.DEALGAE);
        else ScoringManager.endEffector.setIntakeState(IntakeState.HOLD_PIECE);

        /*--------------------------------------------------------------------------------------------------------*/
        // SWERVE
        if (drivingToReef){
            if(SwervePID.atDest() &&  SwervePID.atRot()){
                System.out.println("at dest at rot");
                drivingToReef = !drivingToReef;
            }
            System.out.println("Error: " + SwervePID.getError());
            System.out.println(SwervePID.updateOutputVel());
            SwerveManager.rotateAndDrive(SwervePID.updateOutputRot(), SwervePID.updateOutputVel());
        } else {
            SwerveManager.rotateAndDrive(rotate, drive);
        }


        if (driverStick.getRawButtonPressed(lockIn)) ScoringManager.setScoringLevel(savedLevel);
        if (driverStick.getRawButtonPressed(lockOut)) ScoringManager.setScoringLevel(ScoringPosition.STOW);
    }

    public static void operatorInput() {
        /*-Scoring Manager----------------------------------------------------------------------------------------*/
        if (operatorStick.getRawButtonPressed(stow)) savedLevel = ScoringPosition.STOW;
        else if (operatorStick.getRawButtonPressed(L2)) savedLevel = ScoringPosition.L2;
        else if (operatorStick.getRawButtonPressed(L3)) savedLevel = ScoringPosition.L3;
        else if (operatorStick.getRawButtonPressed(L4)) savedLevel = ScoringPosition.L4;
        else if (operatorStick.getPOV() == ALGAE1) savedLevel = ScoringPosition.ALGAE1;
        else if (operatorStick.getPOV() == ALGAE2) savedLevel = ScoringPosition.ALGAE2;
        else if (operatorStick.getPOV() == LET_HIM_COOK) savedLevel = ScoringPosition.LET_HIM_COOK;


        if (operatorStick.getRawAxis(RightTrigger)>0.7){
            ScoringManager.setPickingRightCoral(true);
        } else if (operatorStick.getRawAxis(LeftTrigger)>0.7){
            ScoringManager.setPickingRightCoral(false);
        }
        
    }

}
