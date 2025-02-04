package frc.robot.swerve.visualizer;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Telemetry;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwerveModule;
import frc.robot.utils.Vector2;

public class SwerveBaseVisualizer {
    private static MechanismRoot2d swerveBase_root = Telemetry.swerveView.getRoot("Swerve Base Root", 30, 30);
    private static MechanismLigament2d swerveBase = swerveBase_root.append(new MechanismLigament2d("Swerve Base", 0, 0));

    private static MechanismRoot2d swerveModule1_root = Telemetry.swerveView.getRoot("Swerve Module 1 Root", 0, 0);
    private static MechanismLigament2d swerveModule1 = swerveModule1_root.append(new MechanismLigament2d("Swerve Module 1", 0, 0));
    private static MechanismRoot2d swerveModule2_root = Telemetry.swerveView.getRoot("Swerve Module 2 Root", 0, 0);
    private static MechanismLigament2d swerveModule2 = swerveModule2_root.append(new MechanismLigament2d("Swerve Module 2", 0, 0));
    private static MechanismRoot2d swerveModule3_root = Telemetry.swerveView.getRoot("Swerve Module 3 Root", 0, 0);
    private static MechanismLigament2d swerveModule3 = swerveModule3_root.append(new MechanismLigament2d("Swerve Module 3", 0, 0));
    private static MechanismRoot2d swerveModule4_root = Telemetry.swerveView.getRoot("Swerve Module 4 Root", 0, 0);
    private static MechanismLigament2d swerveModule4 = swerveModule4_root.append(new MechanismLigament2d("Swerve Module 4", 0, 0));

    private static final double SWERVE_VEL_SCALER = 25.0;

    public static void init() {
        swerveBase.setColor(new Color8Bit(0, 0, 255));
        swerveModule1.setColor(new Color8Bit(0, 255, 0));
        swerveModule2.setColor(new Color8Bit(0, 255, 0));
        swerveModule3.setColor(new Color8Bit(0, 255, 0));
        swerveModule4.setColor(new Color8Bit(0, 255, 0));

        SwerveModule[] mods = SwerveManager.mods;
        swerveModule1_root.setPosition(mods[0].pos.x + 30, mods[0].pos.y + 30);
        swerveModule2_root.setPosition(mods[1].pos.x + 30, mods[1].pos.y + 30);
        swerveModule3_root.setPosition(mods[2].pos.x + 30, mods[2].pos.y + 30);
        swerveModule4_root.setPosition(mods[3].pos.x + 30, mods[3].pos.y + 30);
    }

    public static void update() {
        // Get robot heading
        double heading = Pigeon.getRotationRad();

        // Set swerve base angle and magnitude
        Vector2 swerveVel = SwerveManager.getRobotDriveVelocity();
        swerveBase.setAngle(Math.toDegrees(swerveVel.atan2() + heading));
        swerveBase.setLength(swerveVel.mag() / SWERVE_VEL_SCALER);    

        // get swerve mods from swerve manager
        SwerveModule[] mods = SwerveManager.mods;

        // set mechanism 2d rotations and lengths
        Vector2 module1Pos = mods[0].pos.rotate(heading);
        swerveModule1_root.setPosition(module1Pos.x + 30, module1Pos.y + 30);
        swerveModule1.setAngle(Math.toDegrees(mods[0].getSteerAngle() + heading));
        swerveModule1.setLength(mods[0].getDriveVelocity() / SWERVE_VEL_SCALER);

        Vector2 module2Pos = mods[1].pos.rotate(heading);
        swerveModule2_root.setPosition(module2Pos.x + 30, module2Pos.y + 30);
        swerveModule2.setAngle(Math.toDegrees(mods[1].getSteerAngle() + heading));
        swerveModule2.setLength(mods[1].getDriveVelocity() / SWERVE_VEL_SCALER);

        Vector2 module3Pos = mods[2].pos.rotate(heading);
        swerveModule3_root.setPosition(module3Pos.x + 30, module3Pos.y + 30);
        swerveModule3.setAngle(Math.toDegrees(mods[2].getSteerAngle() + heading));
        swerveModule3.setLength(mods[2].getDriveVelocity() / SWERVE_VEL_SCALER);

        Vector2 module4Pos = mods[3].pos.rotate(heading);
        swerveModule4_root.setPosition(module4Pos.x + 30, module4Pos.y + 30);
        swerveModule4.setAngle(Math.toDegrees(mods[3].getSteerAngle() + heading));
        swerveModule4.setLength(mods[3].getDriveVelocity() / SWERVE_VEL_SCALER);
    }
}
