package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

// AUTO
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
import frc.robot.utils.Vector2;
import frc.robot.vision.VisionManager;

public class Robot extends LoggedRobot {
  @SuppressWarnings("resource")
  public Robot() {
    if (Robot.isReal()){
      try {
        Thread.sleep(5000);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    RTime.init();
    VisionManager.init();
    Auto.init();

    // Swerve
    Pigeon.init();
    SwerveManager.init();
    SwervePosition.init();
    SwervePID.init();
    SwervePosition.setPosition(new Vector2());

    // Subsystems
    ScoringManager.init();
    // Climber.init();

    // Logging
    Telemetry.init();
    
    // Controls
    OI.init();


    Logger.recordMetadata("ProjectName", "2025Reefscape"); // Set a metadata value
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else if (Constants.REPLAY) {
      setUseTiming(true);
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    } else {
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start(); // Start logging

    actuator = new PWM(1);
  }

  @Override
  public void robotPeriodic() {
    RTime.update();
    Telemetry.update();
    SwervePosition.update();
    SwerveManager.update();
    Pigeon.update();
    ScoringManager.update();
  }

  @Override
  public void autonomousInit() {
    Auto.startRoutine();
  }
  
  @Override
  public void autonomousPeriodic() {
    Auto.update();
  }

  PWM actuator;

  @Override
  public void teleopInit() {
    VisionManager.enableLeftCam();
    VisionManager.enableRightCam();
    ScoringManager.setScoringPosition(ScoringPosition.STOW);
  }

  @Override
  public void teleopPeriodic() {
    OI.userInput();
    try {
      actuator.setPosition(0.9);
    } catch (Exception e) {}

  }

  @Override
  public void disabledInit() {
    // Clear Auto Commands
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().disable();

    // Disable Subsystems
    ScoringManager.setScoringPosition(ScoringPosition.DISABLED);
    SwerveManager.rotateAndDrive(0, new Vector2());
    if (Robot.isSimulation()) {
      SwerveManager.mods[0].simModule.speed = 0;
      SwerveManager.mods[1].simModule.speed = 0;
      SwerveManager.mods[2].simModule.speed = 0;
      SwerveManager.mods[3].simModule.speed = 0;
    }

  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
    ScoringManager.setScoringPosition(ScoringPosition.TEST);
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

}
