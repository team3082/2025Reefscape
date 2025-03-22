package frc.robot.subsystems.sim;

import frc.robot.Tuning;
import frc.robot.utils.PIDController;
import frc.robot.utils.RTime;

public class FunnelSim {
    private static PIDController positionPID = new PIDController(
        Tuning.Funnel.CLIMBER_P, 
        Tuning.Funnel.CLIMBER_I, 
        Tuning.Funnel.CLIMBER_D,
        0, 0, 5
    );

    private static double targetPosition;
    private static double position;

    public static void update() {
        position += positionPID.updateOutput(position) * RTime.deltaTime();
    }

    public static void setPosition(double setPosition) {
        if (targetPosition != setPosition){
            targetPosition = setPosition;
            positionPID.setDest(targetPosition);
        }
    }

    public static double getPosition() {
        return position;
    }
}
