package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class Telemetry {
    private static ShuffleboardTab robotTab = Shuffleboard.getTab("SmartDashboard");

    // Views
    private static Field2d fieldView = new Field2d();
    public static Mechanism2d subsytemView = new Mechanism2d(50, 120);
}
