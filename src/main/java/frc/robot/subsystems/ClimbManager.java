package frc.robot.subsystems;

import frc.robot.subsystems.Climber.ClimbState;
import frc.robot.subsystems.Funnel.FunnelState;

public class ClimbManager {
    private enum ClimbingState{
        RESTING, 
        FUNNEL_EXTENDING,
        CLIMBING,
        CLIMBER_RETRACTING,
        FUNNEL_RETRACTING
    }

    private static ClimbingState climbingState;

    public static void init(){
        Funnel.init();
    }

    public static void climb(){
        climbingState = ClimbingState.FUNNEL_EXTENDING;
    }

    public static void retract(){
        climbingState = ClimbingState.CLIMBER_RETRACTING;
    }

    public static void update(){
        switch (climbingState) {
            case RESTING:
                Climber.setState(ClimbState.RESTING);
                Funnel.retract();
                break;
            case FUNNEL_EXTENDING:
                Climber.setState(ClimbState.RESTING);
                Funnel.extend();

                if(Funnel.isExtended()){
                    climbingState = ClimbingState.CLIMBING;
                }
                break;
            case CLIMBING:
                Climber.setState(ClimbState.CLIMBING);
                Funnel.extend();
                break;
            case CLIMBER_RETRACTING:
                Climber.setState(ClimbState.RESTING);
                Funnel.extend();
                if(Climber.climberAtPosition()){
                    climbingState = ClimbingState.FUNNEL_RETRACTING;
                }
                break;
            case FUNNEL_RETRACTING:
                Climber.setState(ClimbState.CLIMBING);
                Funnel.retract();
                if(Funnel.isRetracted()){
                    climbingState = ClimbingState.RESTING;
                }
                break;
            default:
                break;
        }
    }
}
