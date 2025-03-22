package frc.robot.subsystems.visualizer;

import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import frc.robot.Telemetry;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Funnel;

public class FunnelVisualizer {
    private static LoggedMechanismRoot2d root = Telemetry.subsystemView.getRoot("Funnel Root", 1, 1);
    private static LoggedMechanismLigament2d funnelLigament2d = root.append(new LoggedMechanismLigament2d("Funnel", 3, 0));

    public static void init() {
        funnelLigament2d.setLength(Funnel.getPosition()*3);
    }

    public static void update() {
        funnelLigament2d.setLength(Funnel.getPosition()*3);
    }
}