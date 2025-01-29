package frc.robot;

// AUTO
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.Auto;

// SUBSYSTEMS
import frc.robot.subsystems.ScoringManager;
import frc.robot.subsystems.ScoringManager.ScoringPosition;
// import frc.robot.subsystems.AlgaeIntake;
// import frc.robot.subsystems.AlgaeIntake.IntakeState;
// import frc.robot.subsystems.Climber;

public class Robot extends TimedRobot {
  @Override
  public void robotInit() {
    // Controls
    OI.init();

    // Subsystems
    ScoringManager.init();
    // AlgaeIntake.init();
    // Climber.init();

    // Logging
    Telemetry.init();
  }

  @Override
  public void robotPeriodic() {
    // Update Dashboard Every Frame
    Telemetry.update();
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
  }

  @Override
  public void teleopPeriodic() {
    // Handle Input
    OI.userInput();

    // Update Subsystems
    ScoringManager.update();
    // AlgaeIntake.update();
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
