package frc.robot.subsystems;

import frc.robot.Tuning;

/**
 * Manages the scoring system for the robot, including the elevator and end effector.
 */
public class ScoringManager {
    
    /**
     * Enum representing different scoring positions, each with a target height and angle.
     */
    public enum ScoringPosition {
        DISABLED(0.0, 0.0),
        TEST(0.0, 0.0),
        STOW(1.0, 0.0),
        ALGAE1(8.6, Math.PI/3.0),
        ALGAE2(25, Math.PI/3.0),
        L1(13, Math.PI / 2.0 + Math.toRadians(30)),
        L2(15.5, Math.toRadians(30.0)),
        L3(31, Math.toRadians(30.0)),
        L4(62.25, Math.toRadians(60)),
        ALGAE_INTAKE(6.75, Math.PI / 2.0 + Math.toRadians(10));

        public double targetHeight;
        public double targetAngle;
        public boolean isRight;

        /**
         * Constructs a ScoringPosition with the specified target height and angle.
         *
         * @param targetHeight the desired height of the elevator
         * @param targetAngle the desired pivot angle of the end effector
         */
        ScoringPosition(double targetHeight, double targetAngle) {
            this.targetHeight = targetHeight;
            this.targetAngle = targetAngle;
        }
    }
    
    
    /**
     * Enum representing the transitory states when moving between scoring positions.
     */
    public enum TransitoryState {
        ELEVATOR_WAITING, // Elevator waiting for wrist to move to a safe position
        ELEVATOR_MOVING,  // Elevator moving to the target height
        WRIST_MOVING,     // Wrist moving to the correct position after the elevator is set
        FINISHED,         // Movement completed
    }

    public static ScoringPosition scoringPosition = ScoringPosition.STOW;
    public static TransitoryState transitoryState = TransitoryState.FINISHED;
    public static Elevator elevator;
    public static EndEffector endEffector;

    public static Elevator getElevator(){
        return elevator;
    }

    public static EndEffector getEndEffector(){
        return endEffector;
    }

    /**
     * Sets the target scoring position and updates the transitory state if needed.
     *
     * @param targetPosition the desired scoring position
     */
    public static void setScoringPosition(ScoringPosition targetPosition) {
        if (scoringPosition != targetPosition) {
            transitoryState = TransitoryState.ELEVATOR_WAITING;
        }
        scoringPosition = targetPosition;
    }

    public static ScoringPosition getScoringPosition() {
        return scoringPosition;
    }

    /**
     * Creates instances of the elevator and end effector.
     */
    public static void init() {
        elevator = new Elevator();
        endEffector = new EndEffector();
    }

    /**
     * Handles transitions between states.
     */
    public static void update() {
        if (scoringPosition == ScoringPosition.DISABLED) {
            elevator.disable();
            endEffector.disable();
            return;
        } else if (scoringPosition == ScoringPosition.TEST) {
            elevator.test();
            endEffector.test();
            return;
        }

        switch (transitoryState) {
            // moves wrist to safe transitory position
            case ELEVATOR_WAITING:
                handleElevatorWaiting();
                break;
        
            // moves elevator to set position
            case ELEVATOR_MOVING:
                handleElevatorMoving();
                break;
            
            // move wrist to final set position
            case WRIST_MOVING:
                handleWristMoving();
                break;

            // doesn't need anything currently, maybe add manual control if needed
            case FINISHED: 
                break;
        }

        // update end effector and elevator (do not run these methods in Robot.java these should be the only instance)
        endEffector.update();
        elevator.update();
    }

    private static double max_safe_height = 16.0; // inches

    /**
     * Sets the wrist to a safe angle.
     * Transitions to the elevator moving state when the wrist is at the correct position.
     */
    private static void handleElevatorWaiting() {
        endEffector.setPivotAngle(Tuning.EndEffector.SAFE_ANGLE);
        
        if (endEffector.atPosition()) {
            transitoryState = TransitoryState.ELEVATOR_MOVING;
        }

        // make elevator move earlier but constrain it to safe positions
        if (elevator.getElevatorHeight() < max_safe_height + 5.0) {
            double set_height = 0;
            if (scoringPosition.targetHeight < max_safe_height) {
                set_height = scoringPosition.targetHeight;
            } else {
                set_height = max_safe_height;
            }
            elevator.setElevatorHeight(set_height);
        } else {
            double set_height = 0;
            if (scoringPosition.targetHeight > max_safe_height) {
                set_height = scoringPosition.targetHeight;
            } else {
                set_height = max_safe_height;
            }
            elevator.setElevatorHeight(set_height);
        }
    }

    /**
     * Sets the elevator to the target height.
     * Transitions to the wrist moving state when the elevator reaches the correct position.
     */
    private static void handleElevatorMoving() {
        elevator.setElevatorHeight(scoringPosition.targetHeight);
        if (elevator.atPosition()) {
            transitoryState = TransitoryState.WRIST_MOVING;
        }
    }

    /**
     * Sets the wrist to the target angle.
     * Transitions to the finished state when the wrist reaches the correct position.
     */
    private static void handleWristMoving() {
        endEffector.setPivotAngle(scoringPosition.targetAngle);
        if (endEffector.atPosition()) {
            transitoryState = TransitoryState.FINISHED;
        }
    }
}
