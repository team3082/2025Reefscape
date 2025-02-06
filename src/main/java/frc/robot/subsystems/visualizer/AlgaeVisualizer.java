package frc.robot.subsystems.visualizer;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Telemetry;
import frc.robot.subsystems.sim.AlgaeSim;
import frc.robot.utils.RTime;

public class AlgaeVisualizer {
    static MechanismRoot2d algaeRoot = Telemetry.subsytemView.getRoot("root", 30, 1);
    static MechanismLigament2d algaeLigament1 = algaeRoot.append(new MechanismLigament2d("ligament1", 15, 90));
    static MechanismLigament2d wheel = algaeLigament1.append(new MechanismLigament2d("wheel", 1.5, 0));
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
