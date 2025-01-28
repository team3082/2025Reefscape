package frc.robot.subsystems;

import frc.robot.Tuning;

public class ScoringManager {
    
    // TODO set these positions
    public enum ScoringPosition {
        DISABLED(0.0, 0.0),
        STOW(0.0, 0.0),
        INTAKE(0.0, 0.0),
        ALGAE1(0.0, 0.0),
        ALGAE2(0.0, 0.0),
        L1(0.0, 0.0),
        L2(0.0, 0.0),
        L3(0.0, 0.0),
        L4(0.0, 0.0);

        public final double targetHeight;
        public final double targetAngle;

        ScoringPosition(double height, double angle) {
            targetHeight = height;
            targetAngle = angle;
        }
    }

    public enum TransitoryState {
        ELEVATOR_WAITING, // elevator waiting for wrist to move to safe position
        ELEVATOR_MOVING, 
        WRIST_MOVING, // wrist moving to correct position after elevator in correct position
        FINISHED,
    }

    private static ScoringPosition scoringPosition = ScoringPosition.STOW;
    private static TransitoryState transitoryState = TransitoryState.FINISHED;
    private static Elevator elevator;
    private static EndEffector endEffector;

    public static void init() {
        elevator = new Elevator();
        endEffector = new EndEffector();
    }

    public static void setScoringLevel(ScoringPosition targetPosition) {
        if (scoringPosition != targetPosition) {
            transitoryState = TransitoryState.ELEVATOR_WAITING;
        }
        scoringPosition = targetPosition;
    }

    public static ScoringPosition getScoringLevel() {
        return scoringPosition;
    }

    public static void update() {
        switch (transitoryState) {
            case ELEVATOR_WAITING:

            endEffector.setPivotAngle(Tuning.EndEffector.SAFE_ANGLE);

                if (endEffector.atPosition(Tuning.EndEffector.SAFE_ANGLE)) {
                    transitoryState = TransitoryState.ELEVATOR_MOVING;
                }
                
                break;
        
            case ELEVATOR_MOVING:

            elevator.setElevatorHeight(scoringPosition.targetHeight);

                if (elevator.atPosition(scoringPosition.targetHeight)) {
                    transitoryState = TransitoryState.WRIST_MOVING;
                }

                break;

            case WRIST_MOVING:

            endEffector.setPivotAngle(scoringPosition.targetAngle);

                if (endEffector.atPosition(scoringPosition.targetAngle)) {
                    transitoryState = TransitoryState.FINISHED;
                }

                break;

            case FINISHED: // doesn't need anything, maybe add manual control if needed

                break;

        }
    }
}
