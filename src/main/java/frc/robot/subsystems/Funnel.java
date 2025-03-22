package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.sim.ClimberSim;
import frc.robot.subsystems.sim.FunnelSim;
import frc.robot.utils.RTime;

public class Funnel {
    public enum FunnelState {
        RETRACTING(1),
        RESTING(1),
        EXTENDING(0),
        EXTENDED(0);
      

        private double position;
        private FunnelState(double position){
            this.position = position;
        }
    }

    private static FunnelState funnelState;

    private static PWM linearAcutatorLeft;
    private static PWM linearAcutatorRight;

    private static final double TIME_TO_EXTEND = 5;
    private static double endExtendTime = 0;

    private static double position;

    public static void init() {
        linearAcutatorLeft = new PWM(0);
        linearAcutatorRight = new PWM(1);

        linearAcutatorLeft.setPosition(1);
        linearAcutatorRight.setPosition(1);

        funnelState = FunnelState.RESTING;
        position = 1;
    }

    public static void retract(){
        if(funnelState == FunnelState.RESTING || funnelState == FunnelState.RETRACTING ) return ;
    
        funnelState = FunnelState.RETRACTING;
        endExtendTime = RTime.now() + TIME_TO_EXTEND;
    }

    public static void extend(){
        if(funnelState == FunnelState.EXTENDED || funnelState == FunnelState.EXTENDING) return ;
    
        funnelState = FunnelState.EXTENDING;
        endExtendTime = RTime.now() + TIME_TO_EXTEND;
    }

    public static void update(){
        switch (funnelState) {
            case RETRACTING:
                setPosition(funnelState.position);
                if(endExtendTime > RTime.now()){
                    funnelState = FunnelState.EXTENDED;
                }
            break; 
            case RESTING:
                setPosition(funnelState.position);
                break;
            case EXTENDING:
                setPosition(funnelState.position);
                if(endExtendTime > RTime.now()){
                    funnelState = FunnelState.EXTENDED;
                }
                break;
            case EXTENDED:
                setPosition(funnelState.position);
                break;  

            default:
                break;
        }
    }

    public static void setPosition(double position){
        linearAcutatorLeft.setPosition(position);
        linearAcutatorRight.setPosition(position);
    }
    
    
    public static double getPosition() {
        if(Robot.isReal()){
            return position;
        }
        return FunnelSim.getPosition();
    }

    public static boolean isExtended(){
        return funnelState == FunnelState.EXTENDED;
    }

    public static boolean isRetracted(){
        return funnelState == FunnelState.RESTING;
    }

}
