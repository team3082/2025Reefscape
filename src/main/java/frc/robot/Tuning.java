package frc.robot;

import java.util.List;

import frc.robot.utils.Vector2;
import frc.robot.utils.trajectories.QuadraticBezier;
import frc.robot.utils.trajectories.RobotPath;
import frc.robot.vision.AprilTag;
import frc.robot.utils.trajectories.RotPoint;
import frc.robot.utils.trajectories.Curve;
import frc.robot.utils.trajectories.LinearBezier;

public final class Tuning {
    //Swerve
    public static final double MOVEP = Robot.isReal() ? 0.05 : 0.05;
    public static final double MOVEI = 0.0;
    public static final double MOVED = 0.0;
    public static final double MOVEDEAD = Robot.isReal() ? 0.5 : 0.001;
    public static final double MOVEVELDEAD = 0.01;
    public static final double MOVEMAXSPEED = Robot.isReal() ? 0.2 : 0.5;
    // public static final double ROTP = 0.225;
    // public static final double ROTI = 0.0;
    // public static final double ROTD = 0.7;
    public static final double ROTP = 0.3;
    public static final double ROTI = 0.01;
    public static final double ROTD = 0.0;
    public static final double ROTDEAD = 0.05;
    public static final double ROTVELDEAD = 0.01;
    public static final double ROTMAXSPEED = Robot.isReal() ? 0.25 : 0.5;
 
     // TODO Tune
     public static final double SWERVE_TRJ_PPOS = 0.05;
     public static final double SWERVE_TRJ_IPOS = 0.00;
     public static final double SWERVE_TRJ_DPOS = 0.0002;

     public static final double SWERVE_TRJ_PROT = 0.225;
     public static final double SWERVE_TRJ_IROT = 0.0;
     public static final double SWERVE_TRJ_DROT = 0.01;

     public static final double SWERVE_KSPOS = 0.00;
     public static final double SWERVE_KVPOS = 0.85/160;
     public static final double SWERVE_KAPOS = 0.0007;

     public static final double SWERVE_KSROT = 0.00;//0.005;
     public static final double SWERVE_KVROT = 0.0;//0.55 / (3.0 * Math.PI);
     public static final double SWERVE_KAROT = 0.0;
 
     public static final int CURVE_RESOLUTION = 100;
     public static final double CURVE_DEADBAND = 0.75; // bro this is inches who had it at 0.001
    

    //Vision

    public static final class OI {
        //driver
        public static final double KDYAW = 0.00;
        /**0 for never on, 1 for only on with no rotation input, 2 for always on */
        public static final int YAWRATEFEEDBACKSTATUS = 0;

        public static final double NORMALSPEED = 1.0;

        public static final double ROTSPEED = 0.3;
    }

    public static final class Elevator {
        public static final double ELEVATOR_P = 0.25;
        public static final double ELEVATOR_I = 0.0;
        public static final double ELEVATOR_D = 0.005;

        public static final double MOTION_MAGIC_CRUISE_VELOCITY = 0.0;
        public static final double MOTION_MAGIC_ACCELERATION = 0.0;
        public static final double JERK = 0.0;

        public static final double HEIGHT_DEADBAND = 1.0; // in inches
    }

    public static final class EndEffector {
        public static final double PIVOT_P = 0.2;
        public static final double PIVOT_I = 0.0;
        public static final double PIVOT_D = 0.0;

        public static final double MOTION_MAGIC_CRUISE_VELOCITY = 0.0;
        public static final double MOTION_MAGIC_ACCELERATION = 0.0;
        public static final double JERK = 0.0;

        public static final double SAFE_ANGLE = Math.toRadians(28.0); // for moving the elevator so the end effector doesn't hit anything

        public static final double PIVOT_DEADBAND = Math.toRadians(1.0);

        public static final double INTAKE_SPEED = 0.4;
    }
  
    public static final class Intake {
        public static final double STOW_ANGLE = 0.0;
        public static final double FEED_ANGLE = 90.0;
        public static final double HOLD_ANGLE = 0.0;
        public static final double EJECT_ANGLE = 0.0;

        public static final double FEED_SPEED = 800.0;
        public static final double EJECT_SPEED = 0.0;
    }

    public static final class AutoPaths {

        // RIGHT SIDE AUTOPATHS
        public static final Curve START_TO_C = new LinearBezier(Constants.RIGHT_STARTING_POS, Constants.APRIL_TAGS[8].getLeftPosition());
        public static final Curve START_TO_D = new LinearBezier(Constants.RIGHT_STARTING_POS, Constants.APRIL_TAGS[8].getRightPosition());
        public static final Curve START_TO_E = new LinearBezier(Constants.RIGHT_STARTING_POS, Constants.APRIL_TAGS[9].getLeftPosition());
        public static final Curve START_TO_F = new LinearBezier(Constants.RIGHT_STARTING_POS, Constants.APRIL_TAGS[9].getRightPosition());

        public static final Curve STATION_TO_C = new LinearBezier(Constants.CORAL_STATION_RIGHT_POSITION, Constants.APRIL_TAGS[8].getLeftPosition());
        public static final Curve STATION_TO_D = new LinearBezier(Constants.CORAL_STATION_RIGHT_POSITION, Constants.APRIL_TAGS[8].getRightPosition());
        public static final Curve STATION_TO_E = new QuadraticBezier(Constants.CORAL_STATION_RIGHT_POSITION, new Vector2(130, 70), Constants.APRIL_TAGS[9].getLeftPosition());
        public static final Curve STATION_TO_F = new QuadraticBezier(Constants.CORAL_STATION_RIGHT_POSITION, new Vector2(130, 70), Constants.APRIL_TAGS[9].getRightPosition());

        public static final Curve C_TO_STATION = STATION_TO_C.reverse();
        public static final Curve D_TO_STATION = STATION_TO_D.reverse();
        public static final Curve E_TO_STATION = STATION_TO_E.reverse();
        public static final Curve F_TO_STATION = STATION_TO_F.reverse();
        
        // LEFT SIDE AUTOPATHS
        public static final Curve START_TO_L = START_TO_C.flipHorizontal();
        public static final Curve START_TO_K = START_TO_D.flipHorizontal();
        public static final Curve START_TO_J = START_TO_E.flipHorizontal();
        public static final Curve START_TO_I = START_TO_F.flipHorizontal();

        public static final Curve STATION_TO_L = STATION_TO_C.flipHorizontal();
        public static final Curve STATION_TO_K = STATION_TO_D.flipHorizontal();
        public static final Curve STATION_TO_J = STATION_TO_E.flipHorizontal();
        public static final Curve STATION_TO_I = STATION_TO_F.flipHorizontal();

        public static final Curve L_TO_STATION = STATION_TO_L.reverse();
        public static final Curve K_TO_STATION = STATION_TO_K.reverse();
        public static final Curve J_TO_STATION = STATION_TO_J.reverse();
        public static final Curve I_TO_STATION = STATION_TO_I.reverse();

        // MIDDLE AUTOPATHS
        public static final Curve START_TO_G = new LinearBezier(Constants.MIDDLE_STARTING_POS, Constants.APRIL_TAGS[10].getLeftPosition());
        public static final Curve START_TO_H = new LinearBezier(Constants.MIDDLE_STARTING_POS, Constants.APRIL_TAGS[10].getRightPosition());

        public static final Curve G_TO_WAIT = new LinearBezier(Constants.APRIL_TAGS[10].getLeftPosition(), Constants.MIDDLE_WAIT_POS);
        public static final Curve H_TO_WAIT = new LinearBezier(Constants.APRIL_TAGS[10].getRightPosition(), Constants.MIDDLE_WAIT_POS);

        public static final Curve WAIT_TO_BACK_ALGAE = new LinearBezier(Constants.MIDDLE_WAIT_POS, new Vector2(Constants.APRIL_TAGS[10].getPosition().x - 18, Constants.APRIL_TAGS[10].getPosition().y));
        public static final Curve BACK_ALGAE_TO_WAIT = WAIT_TO_BACK_ALGAE.reverse();
    }
}