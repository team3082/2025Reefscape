package frc.robot.subsystems.visualizer;

import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.subsystems.Climber;

public class ClimberVisualizer {
    private static LoggedMechanismRoot2d climberBase_root = Telemetry.subsystemView.getRoot("climber Base Root",
            10 / Constants.METERSTOINCHES, 0 / Constants.METERSTOINCHES);
    private static LoggedMechanismLigament2d climberBase = climberBase_root
            .append(new LoggedMechanismLigament2d("climber Base", 16 / Constants.METERSTOINCHES, 90.0));
    private static LoggedMechanismLigament2d climberHook = climberBase
            .append(new LoggedMechanismLigament2d("climber Hook", 8 / Constants.METERSTOINCHES, 90.0));
    private static LoggedMechanismLigament2d climberHookBend = climberHook
            .append(new LoggedMechanismLigament2d("climber Hook Bend", 6 / Constants.METERSTOINCHES, -30.0));

    public static void init() {
        // set ligament colors
        climberBase.setColor(new Color8Bit(180, 180, 0));
        climberHook.setColor(new Color8Bit(50, 50, 180));
        climberHookBend.setColor(new Color8Bit(50, 50, 180));

    }

    public static void update() {
        // set positions of climber stages
        double climberAngle = Climber.getClimberAngle();
        climberHook.setAngle(Math.toDegrees(climberAngle) - 90);
    }
}