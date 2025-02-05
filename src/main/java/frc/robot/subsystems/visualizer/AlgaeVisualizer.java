package frc.robot.subsystems.visualizer;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.subsystems.sim.AlgaeSim;

public class AlgaeVisualizer {
    public static Mechanism2d mech = new Mechanism2d(10, 20);
    static MechanismRoot2d algaeRoot = mech.getRoot("root", 1, 1);
    static MechanismLigament2d algaeLigament1 = algaeRoot.append(new MechanismLigament2d("ligament1", 5, 90));

    public static void init(){

    }

    public static void update(){
        algaeLigament1.setAngle(AlgaeSim.getAngle());
    }
}
