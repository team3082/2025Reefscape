package frc.robot.swerve.visualizer;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Telemetry;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwerveModule;
import frc.robot.utils.Vector2;

public class SwerveBaseSim {
    private static MechanismRoot2d swerveBase_root = Telemetry.subsytemView.getRoot("Swerve Base Root", 0, 0);
    private static MechanismLigament2d swerveBase = swerveBase_root.append(new MechanismLigament2d("Swerve Base", 0, 0));

    private static MechanismRoot2d swerveModule1_root = Telemetry.subsytemView.getRoot("Swerve Module 1 Root", 0, 0);
    private static MechanismLigament2d swerveModule1 = swerveModule1_root.append(new MechanismLigament2d("Swerve Module 1", 0, 0));
    private static MechanismRoot2d swerveModule2_root = Telemetry.subsytemView.getRoot("Swerve Module 2 Root", 0, 0);
    private static MechanismLigament2d swerveModule2 = swerveModule2_root.append(new MechanismLigament2d("Swerve Module 2", 0, 0));
    private static MechanismRoot2d swerveModule3_root = Telemetry.subsytemView.getRoot("Swerve Module 3 Root", 0, 0);
    private static MechanismLigament2d swerveModule3 = swerveModule3_root.append(new MechanismLigament2d("Swerve Module 3", 0, 0));
    private static MechanismRoot2d swerveModule4_root = Telemetry.subsytemView.getRoot("Swerve Module 4 Root", 0, 0);
    private static MechanismLigament2d swerveModule4 = swerveModule4_root.append(new MechanismLigament2d("Swerve Module 4", 0, 0));

    private static final double SWERVE_VEL_SCALER = 10.0;

    public static void init() {
        swerveBase.setColor(new Color8Bit(0, 0, 255));
        swerveModule1.setColor(new Color8Bit(0, 255, 0));
        swerveModule2.setColor(new Color8Bit(0, 255, 0));
        swerveModule3.setColor(new Color8Bit(0, 255, 0));
        swerveModule4.setColor(new Color8Bit(0, 255, 0));
    }

    public static void update() {
        // Set swerve base angle and magnitude
        Vector2 swerveVel = SwerveManager.getRobotDriveVelocity();
        swerveBase.setAngle(Math.toDegrees(swerveVel.atan2()));
        swerveBase.setLength(swerveVel.mag() / SWERVE_VEL_SCALER);    

        // get swerve mods from swerve manager
        SwerveModule[] mods = SwerveManager.mods;

        // set mechanism 2d rotations and lengths
        swerveModule1.setAngle(Math.toDegrees(mods[0].getSteerAngle()));
        swerveModule1.setLength(Math.toDegrees(mods[0].getDriveVelocity() / SWERVE_VEL_SCALER));
        
        swerveModule2.setAngle(Math.toDegrees(mods[1].getSteerAngle()));
        swerveModule2.setLength(Math.toDegrees(mods[1].getDriveVelocity() / SWERVE_VEL_SCALER));

        swerveModule3.setAngle(Math.toDegrees(mods[2].getSteerAngle()));
        swerveModule3.setLength(Math.toDegrees(mods[2].getDriveVelocity() / SWERVE_VEL_SCALER));

        swerveModule4.setAngle(Math.toDegrees(mods[3].getSteerAngle()));
        swerveModule4.setLength(Math.toDegrees(mods[3].getDriveVelocity() / SWERVE_VEL_SCALER));
    }
}
