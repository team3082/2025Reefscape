package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.CommandAuto;

public class Robot extends TimedRobot {
  @Override
  public void robotPeriodic() {}


  @Override
  public void autonomousInit() {

    CommandScheduler.getInstance().enable();
  }
  
  
  @Override
  public void autonomousPeriodic() {
    CommandAuto.update();
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().disable();
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
