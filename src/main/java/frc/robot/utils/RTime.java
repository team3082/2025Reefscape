package frc.robot.utils;

import java.text.SimpleDateFormat;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class RTime {

    private static double m_startTime;
    private static double m_time;
    private static long m_absoluteFrame;
    // private static double m_deltatime;
    
    // Hardware clock in seconds
    private static double FPGATime() {
        return Timer.getFPGATimestamp();
    }

    /**
     * Zeroes the timer. This should be placed in the teleop and autonomous init functions (and potentially test), 
     * since it's reset at the beginning of these match phases.
     */
    public static void init() {
        m_startTime = FPGATime();
        m_time = 0;
    }

    /**
     * Updates the timer. This should be placed in the teleop and autonomous periodic functions (and potentially test), 
     * since it shouldn't be updated outside of these match phases.
     */
    public static void update() {
        double curtime = FPGATime() - m_startTime;
        // m_deltatime = curtime - m_time; 
        m_time = curtime;
        
    }

    /**
     * Updates the absolute frame. This should be placed in the robot periodic function, since it's meant to keep track
     * of the robot's total uptime in frames.
     */
    public static void updateAbsolute() {
        m_absoluteFrame++;
    }

    /**
     * Returns the current time since init, in seconds.
     */
    public static double now() {
        return m_time;
    }

    /**
     * Returns the total number of timesteps that have passed since the robot was powered on. This should be used sparingly and 
     * only in relative measures; for example, to determine if a set number of timesteps have passed since an action was started. 
     */
    public static long absolute_frame() {
        return m_absoluteFrame;
    }

    /**
     * Returns the current timestamp.
     */
    public static String createTimestamp() {
        return new SimpleDateFormat("MM/dd HH:mm:ss").format(new java.util.Date());
    }

    /**
     * Returns the time since the last time update was called, in seconds.
     */
    public static double deltaTime() {
        //return m_deltatime;
        return Robot.kDefaultPeriod;
    }


}