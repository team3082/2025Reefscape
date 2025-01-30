package frc.robot.subsystems.visualizer;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Telemetry;
import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;
import frc.robot.subsystems.ScoringManager;


/*
 * Visualizer for the end effector using mech 2d, reads values from subsystems and updates the visualizer
 */
public class EndEffectorVisualizer {

    // Wheel positions relative to pivot point
    private static final Vector2 WHEEL_1_POS = new Vector2(0, 12);
    private static final Vector2 WHEEL_2_POS = new Vector2(6, 6);
    private static final Vector2 WHEEL_3_POS = new Vector2(0, 6);
    private static final Vector2 WHEEL_4_POS = new Vector2(6, 12);

    // angles and positions
    private static double pivotHeight = 0;
    private static double pivotAngle = 0;
    private static double wheelAngle = 0;

    // Pivot
    private static MechanismRoot2d endEffectorPivot_root = Telemetry.subsytemView.getRoot("End Effector Pivot Root", 0, 0);
    private static MechanismLigament2d endEffectorPivot = endEffectorPivot_root.append(new MechanismLigament2d("End Effector Pivot", 12, 0));
    
    // Wheels
    private static MechanismRoot2d endEffectorWheel1_root = Telemetry.subsytemView.getRoot("End Effector Wheel 1 Root", 0, 0);
    private static MechanismLigament2d endEffectorWheel1 = endEffectorWheel1_root.append(new MechanismLigament2d("End Effector Wheel 1", 1.5, 0));
    private static MechanismRoot2d endEffectorWheel2_root = Telemetry.subsytemView.getRoot("End Effector Wheel 2 Root", 0, 0);
    private static MechanismLigament2d endEffectorWheel2 = endEffectorWheel2_root.append(new MechanismLigament2d("End Effector Wheel 2", 1.5, 0));
    private static MechanismRoot2d endEffectorWheel3_root = Telemetry.subsytemView.getRoot("End Effector Wheel 3 Root", 0, 0);
    private static MechanismLigament2d endEffectorWheel3 = endEffectorWheel3_root.append(new MechanismLigament2d("End Effector Wheel 3", 1.5, 0));
    private static MechanismRoot2d endEffectorWheel4_root = Telemetry.subsytemView.getRoot("End Effector Wheel 4 Root", 0, 0);
    private static MechanismLigament2d endEffectorWheel4 = endEffectorWheel4_root.append(new MechanismLigament2d("End Effector Wheel 4", 1.5, 0));

    public static void init() {
        // set color to all mechanism ligaments
        endEffectorPivot.setColor(new Color8Bit(255, 0, 0));
        endEffectorWheel1.setColor(new Color8Bit(0, 255, 0));
        endEffectorWheel2.setColor(new Color8Bit(0, 255, 0));
        endEffectorWheel3.setColor(new Color8Bit(0, 255, 0));
        endEffectorWheel4.setColor(new Color8Bit(0, 255, 0));
    }

    public static void update() {
        // set pivot position and angle
        pivotAngle = ScoringManager.endEffector.getPivotAngle();
        pivotHeight = ScoringManager.elevator.getElevatorHeight();
        endEffectorPivot.setAngle(Math.toDegrees(pivotAngle) + 90.0);
        endEffectorPivot_root.setPosition(15, pivotHeight + 8.0);

        // set wheel angles to show rotation
        wheelAngle += RTime.deltaTime() * ScoringManager.endEffector.intakeState.speed * 2.0 * Math.PI * 10.0;
        endEffectorWheel1.setAngle(Math.toDegrees(wheelAngle));
        endEffectorWheel2.setAngle(-Math.toDegrees(wheelAngle));
        endEffectorWheel3.setAngle(-Math.toDegrees(wheelAngle));
        endEffectorWheel4.setAngle(Math.toDegrees(wheelAngle));

        // find and set wheel position with wrist rotation
        Vector2 wheelPos1 = WHEEL_1_POS.rotate(pivotAngle);
        Vector2 wheelPos2 = WHEEL_2_POS.rotate(pivotAngle);
        Vector2 wheelPos3 = WHEEL_3_POS.rotate(pivotAngle);
        Vector2 wheelPos4 = WHEEL_4_POS.rotate(pivotAngle);
        endEffectorWheel1_root.setPosition(wheelPos1.x + 15, wheelPos1.y + pivotHeight + 8.0);
        endEffectorWheel2_root.setPosition(wheelPos2.x + 15, wheelPos2.y + pivotHeight + 8.0);
        endEffectorWheel3_root.setPosition(wheelPos3.x + 15, wheelPos3.y + pivotHeight + 8.0);
        endEffectorWheel4_root.setPosition(wheelPos4.x + 15, wheelPos4.y + pivotHeight + 8.0);
    }
}
