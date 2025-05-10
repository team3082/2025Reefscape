package frc.robot;

import static frc.robot.Tuning.OI.*;

import edu.wpi.first.wpilibj.Joystick;
import frc.controllermaps.LogitechF310;
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
import frc.robot.utils.RTime;

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
    private static final int L1                = LogitechF310.BUTTON_A;
    private static final int L2                  = LogitechF310.BUTTON_B;
    private static final int L3                  = LogitechF310.BUTTON_X;
    private static final int L4                  = LogitechF310.BUTTON_Y;
    private static final int ALGAE1              = LogitechF310.DPAD_DOWN;
    private static final int ALGAE2              = LogitechF310.DPAD_UP;
    private static final int ALGAE_INTAKE        = LogitechF310.DPAD_LEFT;
 


    public static ScoringPosition savedLevel = ScoringPosition.STOW;
    private static double startTime = 0;
    private static double L1ejectStartTime = 0.0;
    private static boolean delaying = false;

    enum AutoAlignState {
        NOT_ALIGNING,
        DRIVING_TO_REEF,
        ELEVATOR_RAISING,
        SCORING,
        SCORINGL4,
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

                if (savedLevel == ScoringPosition.L1) {
                    Vector2 closestPosition;
                    int minIndex = startIndex;

                    closestPosition = Constants.APRIL_TAGS[startIndex].getCenterL1Position();

                    for (int i = startIndex; i < startIndex + 6; i++){
                        Vector2 centerPosition = Constants.APRIL_TAGS[i].getCenterL1Position();
                        Vector2 leftPosition = Constants.APRIL_TAGS[i].getLeftL1Position();
                        Vector2 rightPosition = Constants.APRIL_TAGS[i].getRightL1Position();
    
                        if(currentPos.sub(leftPosition).mag() < currentPos.sub(closestPosition).mag()){
                            minIndex = i;
                            closestPosition = leftPosition;
                        }
                        if(currentPos.sub(centerPosition).mag() < currentPos.sub(closestPosition).mag()){
                            minIndex = i;
                            closestPosition = centerPosition;
                        }
                        if(currentPos.sub(rightPosition).mag() < currentPos.sub(closestPosition).mag()){
                            minIndex = i;
                            closestPosition = rightPosition;
                        }
                    }

                    SwervePID.setDestState(closestPosition, Constants.APRIL_TAGS[minIndex].getRotationZ() + Math.PI / 2.0);
                    aligningState = AutoAlignState.DRIVING_TO_REEF;

                } else {
                
                    // Find the shortest scoring position from the robot
                    double min;
                    min = currentPos.sub(Constants.APRIL_TAGS[startIndex].getCenterPosition()).mag();
                    
                    int minIndex = startIndex;

                    for (int i = startIndex + 1; i < startIndex + 6; i++){
                        Vector2 aprilPosition;
                        aprilPosition = Constants.APRIL_TAGS[i].getCenterPosition();

                        if(currentPos.sub(aprilPosition).mag() < min){
                            minIndex = i;
                            min = currentPos.sub(aprilPosition).mag();
                        }
                    }

                    // Set destination and rotation based on AprilTag data
                    Vector2 targetPosition =  isRight ?  Constants.APRIL_TAGS[minIndex].getRightPosition() : Constants.APRIL_TAGS[minIndex].getLeftPosition();
                    // SwervePID.setDestPt(targetPosition);
                    // SwervePID.setDestRot(Constants.APRIL_TAGS[minIndex].getRotationZ() + Math.PI / 2.0);
                    SwervePID.setDestState(targetPosition, Constants.APRIL_TAGS[minIndex].getRotationZ() + Math.PI / 2.0);
                    aligningState = AutoAlignState.DRIVING_TO_REEF;
                }

            } else {
                aligningState = AutoAlignState.NOT_ALIGNING;
            }
        } else if (!(driverStick.getPOV() == funnyButtonLeft || driverStick.getPOV() == funnyButtonRight)) {
            previouslyPressedPOV = false;
        }

        /*-End Effector-------------------------------------------------------------------------------------------*/

        if (driverStick.getRawAxis(intake) > 0.25) ScoringManager.endEffector.intake();
        else if (driverStick.getRawAxis(outtake) > 0.25) ScoringManager.endEffector.outtake();
        else ScoringManager.endEffector.setIntakeState(IntakeState.HOLD_CORAL);

        if ((ScoringManager.scoringPosition == ScoringPosition.ALGAE1 || ScoringManager.scoringPosition == ScoringPosition.ALGAE2) && ScoringManager.transitoryState == TransitoryState.FINISHED) {
            if (driverStick.getRawAxis(outtake) > 0.25) ScoringManager.endEffector.setPivotAngle(ScoringPosition.ALGAE1.targetAngle);
            else ScoringManager.endEffector.setPivotAngle(Math.PI / 2.0);
        }

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
                delaying = false;
                SwerveManager.rotateAndDrive(rotate, drive);
                break;
        
            case DRIVING_TO_REEF:
                if(SwervePID.atDest() &&  SwervePID.atRot()){
                    aligningState = AutoAlignState.ELEVATOR_RAISING;
                }
                double rotOutput = SwervePID.updateOutputRot();
                Vector2 driveOutput = SwervePID.updateOutputVel();
                SwerveManager.rotateAndDrive(rotOutput, driveOutput);
                ScoringManager.setScoringPosition(savedLevel);
                break;

            case ELEVATOR_RAISING:
                ScoringManager.setScoringPosition(savedLevel);

                if (ScoringManager.transitoryState == TransitoryState.FINISHED) {
                    if(savedLevel == ScoringPosition.L4){
                        startTime = RTime.now();
                        aligningState = AutoAlignState.SCORINGL4;
                    }
                    else aligningState = AutoAlignState.SCORING;
                }
                break;

            case SCORING:
                // if (savedLevel != ScoringPosition.L1) {
                ScoringManager.endEffector.outtake();
                // }
                if (!ScoringManager.endEffector.isHoldingCoral()) {
                    if (savedLevel == ScoringPosition.L1) {
                        if (L1ejectStartTime == 0.0) {
                            L1ejectStartTime = RTime.now();
                        } else if (RTime.now() > L1ejectStartTime + 0.15) {
                            L1ejectStartTime = 0.0;
                            aligningState = AutoAlignState.ELEVATOR_DESCENDING;
                        }
                    } else {
                        aligningState = AutoAlignState.ELEVATOR_DESCENDING;
                    }
                }
                break;
            
            case SCORINGL4:
                ScoringManager.setScoringPosition(ScoringPosition.L4);
                
                if (ScoringManager.getElevator().atPosition() && !delaying) {
                    startTime = RTime.now();
                    delaying = true;
                }
                   
                if(RTime.now() > startTime + 0.1 && delaying){
                    ScoringManager.endEffector.outtake();
                    if (!ScoringManager.endEffector.isHoldingCoral()) {
                        delaying = false;
                        aligningState = AutoAlignState.ELEVATOR_DESCENDING;
                    }
                }
                break;

            case ELEVATOR_DESCENDING:
                ScoringManager.endEffector.setIntakeState(IntakeState.HOLD_CORAL);
                ScoringManager.setScoringPosition(ScoringPosition.STOW);
                aligningState = AutoAlignState.NOT_ALIGNING; // may change later
                drivingToReef = false;
                break;
        }


        if (driverStick.getRawButtonPressed(lockIn)) {
            drivingToReef = false;
            ScoringManager.setScoringPosition(savedLevel);
        }
        if (driverStick.getRawButtonPressed(lockOut)) {
            ScoringManager.setScoringPosition(ScoringPosition.STOW);
            aligningState = AutoAlignState.NOT_ALIGNING;
            drivingToReef = false;
        }
    }

    public static void operatorInput() {
        /*-Scoring Manager----------------------------------------------------------------------------------------*/
        if (operatorStick.getRawButtonPressed(L1)) savedLevel = ScoringPosition.L1;
        else if (operatorStick.getRawButtonPressed(L2)) savedLevel = ScoringPosition.L2;
        else if (operatorStick.getRawButtonPressed(L3)) savedLevel = ScoringPosition.L3;
        else if (operatorStick.getRawButtonPressed(L4)) savedLevel = ScoringPosition.L4;
        else if (operatorStick.getPOV() == ALGAE1) savedLevel = ScoringPosition.ALGAE1;
        else if (operatorStick.getPOV() == ALGAE2) savedLevel = ScoringPosition.ALGAE2;
        else if (operatorStick.getPOV() == ALGAE_INTAKE) savedLevel = ScoringPosition.ALGAE_INTAKE;
    }

}
