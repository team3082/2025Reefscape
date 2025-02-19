package frc.robot.subsystems.visualizer;

import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.subsystems.sim.AlgaeSim;
import frc.robot.utils.RTime;

public class AlgaeVisualizer {
    static LoggedMechanismRoot2d algaeRoot = Telemetry.subsystemView.getRoot("root", (-30/Constants.METERSTOINCHES) + 1.5, 1/Constants.METERSTOINCHES);
    static LoggedMechanismLigament2d algaeLigament1 = algaeRoot.append(new LoggedMechanismLigament2d("ligament1", 15/Constants.METERSTOINCHES, 90));
    static LoggedMechanismLigament2d wheel = algaeLigament1.append(new LoggedMechanismLigament2d("wheel", 1.5/Constants.METERSTOINCHES, 0));
    private static double wheelAngle = 0;


    /**
     * sets the color of the wheel
     */
    public static void init(){
        wheel.setColor(new Color8Bit(255, 255, 255));
    }

    /**
     * updates the angle of the algae ligament and makes the wheel spin
     */
    public static void update(){
        algaeLigament1.setAngle(AlgaeSim.getAngle());

        wheelAngle += AlgaeSim.getSpeed() * RTime.deltaTime();
        wheel.setAngle(wheelAngle);
    }
}
