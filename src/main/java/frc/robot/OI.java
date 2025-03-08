package frc.robot;

import static frc.robot.Tuning.OI.*;

import edu.wpi.first.wpilibj.Joystick;
import frc.controllermaps.LogitechF310;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.ScoringManager;
import frc.robot.subsystems.EndEffector.IntakeState;
import frc.robot.subsystems.ScoringManager.ScoringPosition;
import frc.robot.subsystems.ScoringManager.TransitoryState;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePID;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.Vector2;
import frc.robot.vision.VisionManager;
import frc.robot.utils.RMath;

public class OI {
    public static Joystick driverStick;

    // ------------------ Driver Controls ------------------ //

    // Movement
    public static final int moveX                       = LogitechF310.AXIS_LEFT_X;
    public static final int moveY                       = LogitechF310.AXIS_LEFT_Y;
    public static final int rotateX                     = LogitechF310.AXIS_RIGHT_X;
    
    // static final int boost                       = LogitechF310.AXIS_RIGHT_TRIGGER;
 
    // zero is for Pigeon 
    static final int zero                        = LogitechF310.BUTTON_Y;
 
    static final int funnyButtonLeft             = LogitechF310.BUTTON_LEFT_BUMPER;
    static final int funnyButtonRight            = LogitechF310.BUTTON_RIGHT_BUMPER;
 
    private static final int lockIn              = LogitechF310.BUTTON_X;
    private static final int lockOut             = LogitechF310.BUTTON_B;
 
    // End Effector Control 
    static final int intake                      = LogitechF310.AXIS_LEFT_TRIGGER;
    static final int outtake                     = LogitechF310.AXIS_RIGHT_TRIGGER;
 
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
    private static final int ALGAE_INTAKE        = LogitechF310.DPAD_LEFT;
 


    public static ScoringPosition savedLevel = ScoringPosition.STOW;

    enum AutoAlignState {
        NOT_ALIGNING,
        DRIVING_TO_REEF,
        ELEVATOR_RAISING,
        SCORING,
        ELEVATOR_DESCENDING
    }

    public static AutoAlignState aligningState = AutoAlignState.NOT_ALIGNING;

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

        // double boostStrength = driverStick.getRawAxis(boost);
        // if(boostStrength < 0.1) boostStrength = 0;

        // double kBoostCoefficient = NORMALSPEED + boostStrength * (1.0 - NORMALSPEED);

        /*--------------------------------------------------------------------------------------------------------*/
        // SETUP

        Vector2 drive = new Vector2(driverStick.getRawAxis(moveX) * Math.abs(driverStick.getRawAxis(moveX)), -driverStick.getRawAxis(moveY) * Math.abs(driverStick.getRawAxis(moveY)));
        double rotate =  RMath.smoothJoystick1(driverStick.getRawAxis(rotateX)) * -ROTSPEED;

        if (drive.mag() < 0.05) {
            drive = new Vector2();
        } else {
            drive = RMath.smoothJoystick2(drive);
        }
        if (Math.abs(rotate) < 0.005) {
            rotate = 0;
        }


        /*--------------------------------------------------------------------------------------------------------*/
        // SWERVE

        

        // SCORING
        if ((driverStick.getRawButtonPressed(funnyButtonLeft) || driverStick.getRawButtonPressed(funnyButtonRight)) && !previouslyPressedPOV) {
            drivingToReef = !drivingToReef;
            boolean isRight = (driverStick.getRawButton(funnyButtonRight));
            if(drivingToReef) {
                Vector2 currentPos = SwervePosition.getPosition();
                
                int startIndex = 6;
                
                // Find the shortest scoring position from the robot
                double min;
                if (isRight)
                    min = currentPos.sub(Constants.APRIL_TAGS[startIndex].getRightPosition()).mag();
                else 
                    min = currentPos.sub(Constants.APRIL_TAGS[startIndex].getLeftPosition()).mag();
                
                int minIndex = startIndex;

                for (int i = startIndex + 1; i < startIndex + 6; i++){
                    Vector2 aprilPosition;
                    if (isRight)
                        aprilPosition = Constants.APRIL_TAGS[i].getRightPosition();
                    else 
                        aprilPosition = Constants.APRIL_TAGS[i].getLeftPosition();

                    if(currentPos.sub(aprilPosition).mag() < min){
                        minIndex = i;
                        min = currentPos.sub(aprilPosition).mag();
                    }
                }

                // Set destination and rotation based on AprilTag data
                Vector2 targetPosition =  isRight ?  Constants.APRIL_TAGS[minIndex].getRightPosition() : Constants.APRIL_TAGS[minIndex].getLeftPosition();
                SwervePID.setDestPt(targetPosition);
                SwervePID.setDestRot(Constants.APRIL_TAGS[minIndex].getRotationZ() + Math.PI / 2.0);

                aligningState = AutoAlignState.DRIVING_TO_REEF;

            }
        } else if (!(driverStick.getPOV() == funnyButtonLeft || driverStick.getPOV() == funnyButtonRight)) {
            previouslyPressedPOV = false;
        }

        /*-End Effector-------------------------------------------------------------------------------------------*/

        if (driverStick.getRawAxis(intake) > 0.25) ScoringManager.endEffector.intake();
        else if (driverStick.getRawAxis(outtake) > 0.25) ScoringManager.endEffector.outtake();
        else ScoringManager.endEffector.setIntakeState(IntakeState.HOLD_CORAL);

        /*--------------------------------------------------------------------------------------------------------*/
        // SWERVE
        // if (drivingToReef){
        //     // VisionManager.disableVision();
        //     if(SwervePID.atDest() &&  SwervePID.atRot()){
        //         System.out.println("at dest at rot");
        //         drivingToReef = !drivingToReef;
        //     }
        //     double rotOutput = SwervePID.updateOutputRot();
        //     Vector2 driveOutput = SwervePID.updateOutputVel();
        //     System.out.println("rot output: " + rotOutput + " drive output: " + driveOutput.toString());
        //     SwerveManager.rotateAndDrive(rotOutput, driveOutput);
        // } else {
        //     VisionManager.enableVision();
        //     SwerveManager.rotateAndDrive(rotate, drive);
        // }
        VisionManager.enableVision();
        switch (aligningState) {
            case NOT_ALIGNING:
                if (driverStick.getRawButton(intake)) ScoringManager.endEffector.intake();
                else if (driverStick.getRawButton(outtake)) ScoringManager.endEffector.outtake();
                else ScoringManager.endEffector.setIntakeState(IntakeState.HOLD_CORAL);
                SwerveManager.rotateAndDrive(rotate, drive);
                break;
        
            case DRIVING_TO_REEF:
                System.out.println("Driving to Reef");
                if(SwervePID.atDest() &&  SwervePID.atRot()){
                    aligningState = AutoAlignState.ELEVATOR_RAISING;
                }
                double rotOutput = SwervePID.updateOutputRot();
                Vector2 driveOutput = SwervePID.updateOutputVel();
                SwerveManager.rotateAndDrive(rotOutput, driveOutput);
                break;

            case ELEVATOR_RAISING:
                System.out.println("Elevator Raising");
                ScoringManager.setScoringPosition(savedLevel);
                System.out.println("Saved Level: " + savedLevel.toString());
                if (ScoringManager.transitoryState == TransitoryState.FINISHED) {
                    aligningState = AutoAlignState.SCORING;
                }
                break;

            case SCORING:
                System.out.println("Scoring");
                ScoringManager.endEffector.outtake();
                if (!ScoringManager.endEffector.isHoldingCoral()) {
                    aligningState = AutoAlignState.ELEVATOR_DESCENDING;
                }
                break;

            case ELEVATOR_DESCENDING:
                System.out.println("Elevator Descending");
                ScoringManager.endEffector.setIntakeState(IntakeState.HOLD_CORAL);
                ScoringManager.setScoringPosition(ScoringPosition.STOW);
                aligningState = AutoAlignState.NOT_ALIGNING; // may change later
                break;
        }


        if (driverStick.getRawButtonPressed(lockIn)) {
            drivingToReef = false;
            ScoringManager.setScoringPosition(savedLevel);
        }
        if (driverStick.getRawButtonPressed(lockOut)) ScoringManager.setScoringPosition(ScoringPosition.STOW);
    }

    public static void operatorInput() {
        /*-Scoring Manager----------------------------------------------------------------------------------------*/
        if (operatorStick.getRawButtonPressed(stow)) savedLevel = ScoringPosition.STOW;
        else if (operatorStick.getRawButtonPressed(L2)) savedLevel = ScoringPosition.L2;
        else if (operatorStick.getRawButtonPressed(L3)) savedLevel = ScoringPosition.L3;
        else if (operatorStick.getRawButtonPressed(L4)) savedLevel = ScoringPosition.L4;
        else if (operatorStick.getPOV() == ALGAE1) savedLevel = ScoringPosition.ALGAE1;
        else if (operatorStick.getPOV() == ALGAE2) savedLevel = ScoringPosition.ALGAE2;
        else if (operatorStick.getPOV() == ALGAE_INTAKE) savedLevel = ScoringPosition.ALGAE_INTAKE;
    }

}
