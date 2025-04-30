package frc.robot;


import frc.robot.Constants.REEF_POSITIONS;
import frc.robot.utils.Vector2;
import frc.robot.utils.trajectories.QuadraticBezier;
import frc.robot.utils.trajectories.Curve;
import frc.robot.utils.trajectories.LinearBezier;

public final class Tuning {
    //Swerve
    public static final double MOVEP = 0.0375;
    public static final double MOVEI = 0.0001;
    public static final double MOVED = 0.004;
    public static final double MOVEDEAD = 0.5;
    public static final double MOVEVELDEAD = 0.05;
    public static final double MOVEMAXSPEED = 0.3;
    
    public static final double ROTP = 0.4;
    public static final double ROTI = 0.05;
    public static final double ROTD = 0.075;
    public static final double ROTDEAD = 0.02;
    public static final double ROTVELDEAD = 0.05;
    public static final double ROTMAXSPEED = 0.3;

 
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
     public static final double CURVE_DEADBAND = 0.5; // bro this is inches who had it at 0.001
     public static final double ROT_DEADBAND = 0.03; // radians
    

    // Trapezoidal Tuning
    public static final double MOVE_PRECISE_VEL = 16 * 12;
    public static final double MOVE_PRECISE_ACC = 8 * 12;
    public static final double MOVE_PRECISE_DEC = 8 * 12;

    public static final double MOVE_FAST_VEL = 16 * 12;
    public static final double MOVE_FAST_ACC = 10 * 12;
    public static final double MOVE_FAST_DEC = 10 * 12;

    public static final double ROT_PRECISE_VEL = 1.0;
    public static final double ROT_PRECISE_ACC = 0.5;
    public static final double ROT_PRECISE_DEC = 0.5;

    public static final double ROT_FAST_VEL = 1.0;
    public static final double ROT_FAST_ACC = 0.5;
    public static final double ROT_FAST_DEC = 0.5;

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
        public static final double ELEVATOR_P = 0.35;
        public static final double ELEVATOR_I = 0.0;
        public static final double ELEVATOR_D = 0.005;
        public static final double ELEVATOR_G = 0.05; // feed forward

        public static final double MOTION_MAGIC_CRUISE_VELOCITY = 1500;
        public static final double MOTION_MAGIC_ACCELERATION = 750;
        public static final double MOTION_MAGIC_JERK = 1500;

        public static final double HEIGHT_DEADBAND = 1.0; // in inches
    }

    public static final class EndEffector {
        public static final double PIVOT_P = 0.2;
        public static final double PIVOT_I = 0.0;
        public static final double PIVOT_D = 0.0;

        public static final double MOTION_MAGIC_CRUISE_VELOCITY = 600;
        public static final double MOTION_MAGIC_ACCELERATION = 300;
        public static final double MOTION_MAGIC_JERK = 0.0;

        public static final double SAFE_ANGLE = Math.toRadians(30.0); // for moving the elevator so the end effector doesn't hit anything

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
        public static final Curve START_TO_C = new LinearBezier(Constants.RIGHT_STARTING_POS, Constants.REEF_POSITIONS.C.getPosition());
        public static final Curve START_TO_D = new LinearBezier(Constants.RIGHT_STARTING_POS, Constants.REEF_POSITIONS.D.getPosition());
        public static final Curve START_TO_E = new LinearBezier(Constants.RIGHT_STARTING_POS, Constants.REEF_POSITIONS.E.getPosition());
        public static final Curve START_TO_F = new LinearBezier(Constants.RIGHT_STARTING_POS, Constants.REEF_POSITIONS.F.getPosition());

        public static final Curve STATION_TO_C = new LinearBezier(Constants.CORAL_STATION_RIGHT_POSITION, Constants.REEF_POSITIONS.C.getPosition());
        public static final Curve STATION_TO_D = new LinearBezier(Constants.CORAL_STATION_RIGHT_POSITION, Constants.REEF_POSITIONS.D.getPosition());
        public static final Curve STATION_TO_E = new QuadraticBezier(Constants.CORAL_STATION_RIGHT_POSITION, new Vector2(130, 70), Constants.REEF_POSITIONS.E.getPosition());
        public static final Curve STATION_TO_F = new QuadraticBezier(Constants.CORAL_STATION_RIGHT_POSITION, new Vector2(130, 70), Constants.REEF_POSITIONS.F.getPosition());
        public static final Curve STATION_TO_B = new LinearBezier(Constants.CORAL_STATION_RIGHT_POSITION, Constants.REEF_POSITIONS.B.getPosition());
        public static final Curve STATION_TO_G = new QuadraticBezier(Constants.CORAL_STATION_RIGHT_POSITION, new Vector2(95, 95), Constants.REEF_POSITIONS.G.getPosition());

        public static final Curve C_TO_STATION = STATION_TO_C.reverse();
        public static final Curve D_TO_STATION = STATION_TO_D.reverse();
        public static final Curve E_TO_STATION = STATION_TO_E.reverse();
        public static final Curve F_TO_STATION = STATION_TO_F.reverse();
        public static final Curve B_TO_STATION = STATION_TO_B.reverse();
        public static final Curve G_TO_STATION = STATION_TO_G.reverse();
        
        // LEFT SIDE AUTOPATHS
        public static final Curve START_TO_L = START_TO_C.flipHorizontal();
        public static final Curve START_TO_K = START_TO_D.flipHorizontal();
        public static final Curve START_TO_J = START_TO_E.flipHorizontal();
        public static final Curve START_TO_I = START_TO_F.flipHorizontal();

        public static final Curve STATION_TO_L = STATION_TO_C.flipHorizontal();
        public static final Curve STATION_TO_K = STATION_TO_D.flipHorizontal();
        public static final Curve STATION_TO_J = STATION_TO_E.flipHorizontal();
        public static final Curve STATION_TO_I = STATION_TO_F.flipHorizontal();
        public static final Curve STATION_TO_A = STATION_TO_B.flipHorizontal();
        public static final Curve STATION_TO_H = STATION_TO_G.flipHorizontal();

        public static final Curve L_TO_STATION = STATION_TO_L.reverse();
        public static final Curve K_TO_STATION = STATION_TO_K.reverse();
        public static final Curve J_TO_STATION = STATION_TO_J.reverse();
        public static final Curve I_TO_STATION = STATION_TO_I.reverse();
        public static final Curve A_TO_STATION = STATION_TO_A.reverse();
        public static final Curve H_TO_STATION = STATION_TO_H.reverse();

        // MIDDLE AUTOPATHS
        public static final Curve START_TO_G = new LinearBezier(Constants.MIDDLE_STARTING_POS, Constants.REEF_POSITIONS.G.getPosition());
        public static final Curve START_TO_H = new LinearBezier(Constants.MIDDLE_STARTING_POS, Constants.REEF_POSITIONS.H.getPosition());

        public static final Curve G_TO_WAIT = new LinearBezier(Constants.REEF_POSITIONS.G.getPosition(), Constants.MIDDLE_WAIT_POS);
        public static final Curve H_TO_WAIT = new LinearBezier(Constants.REEF_POSITIONS.H.getPosition(), Constants.MIDDLE_WAIT_POS);

        public static final Curve WAIT_TO_BACK_ALGAE = new LinearBezier(Constants.MIDDLE_WAIT_POS, new Vector2(REEF_POSITIONS.G.redTag.getCenterPosition().x - 18, REEF_POSITIONS.G.redTag.getCenterPosition().y));
        public static final Curve BACK_ALGAE_TO_WAIT = WAIT_TO_BACK_ALGAE.reverse();

        public static Curve getAutoPath(Curve path) {
            return path;
        }
    }
}