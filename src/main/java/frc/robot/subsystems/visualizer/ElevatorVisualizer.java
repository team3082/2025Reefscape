package frc.robot.subsystems.visualizer;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Telemetry;
import frc.robot.subsystems.LebronJames;

/*
 * Visualizer for the elevator using mech 2d, reads values from subsystems and updates the visualizer
 */
public class ElevatorVisualizer {
    private static MechanismRoot2d elevatorBase_root = Telemetry.subsytemView.getRoot("Elevator Base Root", 26, 0);
    private static MechanismRoot2d elevatorStage1_root = Telemetry.subsytemView.getRoot("Elevator Stage 1 Root", 24, 0);
    private static MechanismRoot2d elevatorStage2_root = Telemetry.subsytemView.getRoot("Elevator Stage 2 Root", 22, 0);
    private static MechanismRoot2d elevatorStage3_root = Telemetry.subsytemView.getRoot("Elevator Stage 3 Root", 20, 0);

    private static MechanismLigament2d elevatorBase = elevatorBase_root.append(new MechanismLigament2d("Elevator Base", 32, 90.0));
    private static MechanismLigament2d elevatorStage1 = elevatorStage1_root.append(new MechanismLigament2d("Elevator Stage 1", 33, 90.0));
    private static MechanismLigament2d elevatorStage2 = elevatorStage2_root.append(new MechanismLigament2d("Elevator Stage 2", 33, 90.0));
    private static MechanismLigament2d elevatorStage3 = elevatorStage3_root.append(new MechanismLigament2d("Elevator Stage 3", 8, 90.0));

    /**
     * sets ligament colors
     */
    public static void init() {
        elevatorBase.setColor(new Color8Bit(0, 0, 255));
        elevatorStage1.setColor(new Color8Bit(0, 0, 255));
        elevatorStage2.setColor(new Color8Bit(0, 0, 255));
        elevatorStage3.setColor(new Color8Bit(0, 0, 255));
    }

    /**
     * sets positions of elevator stages
     */
    public static void update() {
        double elevatorHeight = LebronJames.elevator.getElevatorHeight();
        elevatorStage1_root.setPosition(24  , elevatorHeight / 3.0);
        elevatorStage2_root.setPosition(22, elevatorHeight / (3.0 / 2.0));
        elevatorStage3_root.setPosition(20, elevatorHeight);
    }
}
