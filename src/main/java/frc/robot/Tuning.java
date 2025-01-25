package frc.robot;

public final class Tuning {
    //Swerve
    public static final double MOVEP = 2;
    public static final double MOVEI = 0.3;
    public static final double MOVED = 0.2;
    public static final double MOVEDEAD = 1.0;
    public static final double MOVEVELDEAD = 0.0;
    public static final double MOVEMAXSPEED = 0.0;
    public static final double ROTP = 0.225;
    public static final double ROTI = 0.0;
    public static final double ROTD = 0.7;
    public static final double ROTDEAD = 0.025;
    public static final double ROTVELDEAD = 0.01;
    public static final double ROTMAXSPEED = 0.5;
 
     // TODO Tune
     public static final double SWERVE_TRJ_PPOS = 0.005;
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
 
     public static final int CURVE_RESOLUTION = 3000;
     public static final double CURVE_DEADBAND = 0.001;
    

    //Vision

    public static final class OI {
        //driver
        public static final double KDYAW = 0.00;
        /**0 for never on, 1 for only on with no rotation input, 2 for always on */
        public static final int YAWRATEFEEDBACKSTATUS = 0;

        public static final double NORMALSPEED = 0.4;

        public static final double ROTSPEED = 0.3;
    }

    public static final class Elevator {
        public static final double ELEVATOR_P = 0.0;
        public static final double ELEVATOR_I = 0.0;
        public static final double ELEVATOR_D = 0.0;

        public static final double MOTION_MAGIC_CRUISE_VELOCITY = 0.0;
        public static final double MOTION_MAGIC_ACCELERATION = 0.0;
        public static final double JERK = 0.0;

        public static final double HEIGHT_DEADBAND = 1.0; // in inches
    }

    public static final class EndEffector {
        public static final double PIVOT_P = 0.0;
        public static final double PIVOT_I = 0.0;
        public static final double PIVOT_D = 0.0;

        public static final double MOTION_MAGIC_CRUISE_VELOCITY = 0.0;
        public static final double MOTION_MAGIC_ACCELERATION = 0.0;
        public static final double JERK = 0.0;

        public static final double SAFE_ANGLE = 0.0; // for moving the elevator so the end effector doesn't hit anything

        public static final double PIVOT_DEADBAND = Math.toRadians(1.0);
    }
  
    public static final class Intake {
        public static final double STOW_ANGLE = 0.0;
        public static final double FEED_ANGLE = 0.0;
        public static final double HOLD_ANGLE = 0.0;
        public static final double EJECT_ANGLE = 0.0;

        public static final double FEED_SPEED = 0.0;
        public static final double EJECT_SPEED = 0.0;
    }
}