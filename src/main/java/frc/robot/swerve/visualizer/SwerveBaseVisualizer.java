package frc.robot.swerve.visualizer;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Telemetry;
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

    private static final double SWERVE_VEL_SCALER = 100.0;

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
        // Set swerve base angle and magnitude
        Vector2 swerveVel = SwerveManager.getRobotDriveVelocity();
        swerveBase.setAngle(Math.toDegrees(swerveVel.atan2() - (Math.PI / 2.0)));
        swerveBase.setLength(swerveVel.mag());    

        // get swerve mods from swerve manager
        SwerveModule[] mods = SwerveManager.mods;

        // set mechanism 2d rotations and lengths
        swerveModule1.setAngle(Math.toDegrees(mods[0].getSteerAngle() - (Math.PI / 2.0)));
        swerveModule1.setLength(Math.toDegrees(mods[0].getDriveVelocity() / SWERVE_VEL_SCALER * (mods[0].inverted ? -1 : 1)));
        
        swerveModule2.setAngle(Math.toDegrees(mods[1].getSteerAngle() - (Math.PI / 2.0)));
        swerveModule2.setLength(Math.toDegrees(mods[1].getDriveVelocity() / SWERVE_VEL_SCALER * (mods[1].inverted ? -1 : 1)));

        swerveModule3.setAngle(Math.toDegrees(mods[2].getSteerAngle() - (Math.PI / 2.0)));
        swerveModule3.setLength(Math.toDegrees(mods[2].getDriveVelocity() / SWERVE_VEL_SCALER * (mods[2].inverted ? -1 : 1)));

        swerveModule4.setAngle(Math.toDegrees(mods[3].getSteerAngle() - (Math.PI / 2.0)));
        swerveModule4.setLength(Math.toDegrees(mods[3].getDriveVelocity() / SWERVE_VEL_SCALER * (mods[3].inverted ? -1 : 1)));
    }
}
