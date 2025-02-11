package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
// AUTO
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.Auto;

// SUBSYSTEMS
import frc.robot.subsystems.ScoringManager;
import frc.robot.subsystems.ScoringManager.ScoringPosition;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePID;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.RTime;

public class Robot extends LoggedRobot {

  @AutoLogOutput
  public static double fake = 0;

  public Robot(){
    try {
      Thread.sleep(5000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    // Swerve
    Pigeon.init();
    Pigeon.setYaw(90);
    SwerveManager.init();
    SwervePosition.init();
    SwervePID.init();
    SwervePosition.setPosition(Constants.APRIL_TAGS[7].getPosition());

    // Subsystems
    // ScoringManager.init();
    // AlgaeIntake.init();
    // Climber.init();


    // Auto.init();

    // Controls
    OI.init();

    RTime.init();

    // Logging
    Telemetry.init();

    //Pigeon.setYaw(90);
    Logger.recordMetadata("ProjectName", "2025Reefscape"); // Set a metadata value
    Logger.recordOutput("PID Error", SwervePID.getError().toString());

    AutoLogOutputManager.addPackage("frc.robot.swerve");
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else if (Constants.REPLAY) {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    } else {
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start(); // Start logging
  }

  @Override
  public void robotInit() {
    // try {
    //   Thread.sleep(5000);
    // } catch (InterruptedException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }

    // // Swerve
    // Pigeon.init();
    // Pigeon.setYaw(90);
    // SwerveManager.init();
    // SwervePosition.init();
    // SwervePID.init();
    // //SwervePosition.setPosition(Constants.APRIL_TAGS[17].getPosition());

    // // Subsystems
    // // ScoringManager.init();
    // // AlgaeIntake.init();
    // // Climber.init();


    // // Auto.init();

    // // Controls
    // OI.init();

    // RTime.init();

    // // Logging
    // Telemetry.init();

    // //Pigeon.setYaw(90);
  }

  @Override
  public void robotPeriodic() {
    // Update Dashboard Every Frame
    Telemetry.update();

    SwervePosition.update();
    Pigeon.update();

    RTime.update();
  }

  @Override
  public void autonomousInit() {
    // CommandScheduler.getInstance().enable();
    // Auto.autoInit();
  }
  
  @Override
  public void autonomousPeriodic() {
    // Auto.update();
  }

  @Override
  public void teleopInit() {
    // Stow Elevator/End Effector on Teleop Start
    // ScoringManager.setScoringLevel(ScoringPosition.STOW);
  }

  @Override
  public void teleopPeriodic() {
    // Handle Input
    OI.userInput();

    // Update Subsystems
    SwerveManager.update();
    // ScoringManager.update();
    // AlgaeIntake.update();
    // Climber.update();
  }

  @Override
  public void disabledInit() {
    // Clear Auto Commands
    // CommandScheduler.getInstance().cancelAll();
    // CommandScheduler.getInstance().disable();

    // Disable Subsystems
    // ScoringManager.setScoringLevel(ScoringPosition.DISABLED);
    // AlgaeIntake.setState(IntakeState.DISABLED);
    // Climber.setState(ClimberState.DISABLED);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
