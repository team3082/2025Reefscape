package frc.robot.auto;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.OpenHouseComplier.AutoRoutine;

/** Class for selecting autonomous routines. */
public class AutoSelector {

  // If you want to know what auto is running anywhere else in the code
  public static String selectedAuto;

  public static  SendableChooser<String> autoChooser = new SendableChooser<>();
  private static HashMap<String, AutoRoutine> allRoutines;

  /*
   * Setup the Auto selector.
   */
  public static void setup() {
    //I apolgize in advance for what I am about to write
    allRoutines = OpenHouseComplier.getHash();
    autoChooser = OpenHouseComplier.autoChooser;
  }

  /**
   * Run the auto selector. Gets the selected string from Glass, and runs the auto that corresponds
   * with the string chosen.
   */
  public static void run() {
    CommandAuto.init(allRoutines.get(autoChooser.getSelected()).getCommands());
  }
}
