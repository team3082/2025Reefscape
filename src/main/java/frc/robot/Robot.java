package frc.robot;

// AUTO
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.Auto;
import frc.robot.subsystems.AlgaeIntake;
// SUBSYSTEMS
import frc.robot.subsystems.ScoringManager;
import frc.robot.subsystems.AlgaeIntake.IntakeState;
import frc.robot.subsystems.ScoringManager.ScoringPosition;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePID;
// import frc.robot.subsystems.AlgaeIntake;
// import frc.robot.subsystems.AlgaeIntake.IntakeState;
// import frc.robot.subsystems.Climber;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.RTime;

public class Robot extends TimedRobot {
  @Override
  public void robotInit() {
    // Swerve
    SwerveManager.init();
    SwervePosition.init();
    SwervePID.init();
    Pigeon.init();


    // Subsystems
    ScoringManager.init();
    AlgaeIntake.init();
    // Climber.init();

    // Logging
    Telemetry.init();
    
    // Controls
    OI.init();

    RTime.init();
  }

  @Override
  public void robotPeriodic() {
    // Update Dashboard Every Frame
    Telemetry.update();

    SwervePosition.update();
    Pigeon.update();
  }

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().enable();
  }
  
  @Override
  public void autonomousPeriodic() {
    Auto.update();
  }

  @Override
  public void teleopInit() {
    // Stow Elevator/End Effector on Teleop Start
    ScoringManager.setScoringLevel(ScoringPosition.STOW);
    AlgaeIntake.setState(IntakeState.FEED);
  }

  @Override
  public void teleopPeriodic() {
    // Handle Input
    OI.userInput();

    // Update Subsystems
    SwerveManager.update();
    ScoringManager.update();
    AlgaeIntake.update();
    // Climber.update();
  }

  @Override
  public void disabledInit() {
    // Clear Auto Commands
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().disable();

    // Disable Subsystems
    ScoringManager.setScoringLevel(ScoringPosition.DISABLED);
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
