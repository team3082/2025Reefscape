package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.vision.AprilTag;

public class Constants {
    
    public static final AprilTag[] APRIL_TAGS = {
        null,
        // Red Alliance
        new AprilTag(311.93, -132.7, 58.5, Math.toRadians(126), 0),                    // id 1
        new AprilTag(311.93, 132.7, 58.5, Math.toRadians(234), 0),                   // id 2
        new AprilTag(109.71, 158.65, 51.25, Math.toRadians(270), 0),                 // id 3
        new AprilTag(19.76, 83.14, 73.54, 0, Math.toRadians(30)),                    // id 4
        new AprilTag(19.76, -83.11, 73.54, 0, Math.toRadians(30)),                     // id 5
        new AprilTag(185.05, -28.33, 12.13, Math.toRadians(300), 0),                   // id 6
        new AprilTag(201.43, 0, 12.13, Math.toRadians(0), 0),                        // id 7
        new AprilTag(185.05, 28.33, 12.13, Math.toRadians(60), 0),                   // id 8
        new AprilTag(152.33, 28.33, 12.13, Math.toRadians(120), 0),                  // id 9
        new AprilTag(135.95, 0, 12.13, Math.toRadians(180), 0),                      // id 10
        new AprilTag(152.33, -28.33, 12.13, Math.toRadians(240), 0),                   // id 11

        // Blue Alliance
        new AprilTag(-311.93, -132.7, 58.5, Math.toRadians(54), 0),                      // id 12
        new AprilTag(-311.93, 132.7, 58.5, Math.toRadians(306), 0),                    // id 13
        new AprilTag(-19.76, 83.14, 73.54, Math.toRadians(180), Math.toRadians(30)), // id 14
        new AprilTag(-19.76, -83.11, 73.54, Math.toRadians(180), Math.toRadians(30)),  // id 15
        new AprilTag(-109.71, -158.65, 51.25, Math.toRadians(90), 0),                    // id 16
        new AprilTag(-185.05, -28.33, 12.13, Math.toRadians(240), 0),                    // id 17
        new AprilTag(-201.43, 0, 12.13, Math.toRadians(180), 0),                       // id 18
        new AprilTag(-185.05, 28.33, 12.13, Math.toRadians(120), 0),                   // id 19
        new AprilTag(-152.33, 28.33, 12.13, Math.toRadians(60), 0),                    // id 20
        new AprilTag(-135.95, 0, 12.13, Math.toRadians(0), 0),                         // id 21
        new AprilTag(-152.33, -28.33, 12.13, Math.toRadians(300), 0),                    // id 22
        
    };

    public static final class Elevator {
        public static final int MOTORID1 = 0;
        public static final int MOTORID2 = 0;

        public static final double GEARRATIO = 25.0; 
        public static final double INCHESPERROTATION = 2.865 * GEARRATIO / 3.0; // sprocket chain pitch diameter * gear ratio
    }

    public static final class EndEffector {
        public static final int PIVOTID = 0;
        public static final int END_EFFECTOR_SENSOR_ID = 0;

        public static final double GEARRATIO = 0.0;
    }

    public static final class Swerve {

        public static final int DRIVEID0 = 5;
        public static final int DRIVEID1 = 7;
        public static final int DRIVEID2 = 1;
        public static final int DRIVEID3 = 3;

        public static final int STEERID0 = 6;
        public static final int STEERID1 = 8;
        public static final int STEERID2 = 2;
        public static final int STEERID3 = 4;

        public static final double MODOFFSET0 = 286.348;
        public static final double MODOFFSET1 = 221.309;
        public static final double MODOFFSET2 = 174.551;
        public static final double MODOFFSET3 = 134.736;

        public static final double WIDTH = 26;
        public static final double LENGTH = 26;
        public static final double MODULEOFFSET = 2.625;

        public static final double SWERVEMODX0 = (WIDTH / 2) - MODULEOFFSET;
        public static final double SWERVEMODX1 = -1 * (WIDTH / 2) + MODULEOFFSET;
        public static final double SWERVEMODX2 = -1 * (WIDTH / 2) + MODULEOFFSET;
        public static final double SWERVEMODX3 = (WIDTH / 2) - MODULEOFFSET;

        public static final double SWERVEMODY0 = -1 * (LENGTH / 2) + MODULEOFFSET;
        public static final double SWERVEMODY1 = -1 * (LENGTH / 2) + MODULEOFFSET;
        public static final double SWERVEMODY2 = (LENGTH / 2) - MODULEOFFSET;
        public static final double SWERVEMODY3 = (LENGTH / 2) - MODULEOFFSET;

        public static final double driveTrackwidth = 0.0;
        public static final double driveWheelbase = 0.0;

        public static final double shootWhileMoveSpeed = 0.24;
    
        // The unadjusted maximum velocity of the robot, in inches per second.
        public static final double maxChassisVelocity = 6380.0 / 60.0 * 6.12 * (4.0 * Math.PI);
        // The unadjusted theoretical maximum angular velocity of the robot, in radians per second.
        public static final double maxAngularVelocity = (maxChassisVelocity / Math.hypot(driveTrackwidth / 2.0, driveWheelbase / 2.0));

        // Rotations to Radians
        public static final double rotConversionFactor = 2 * Math.PI;

        public static final class MK4iDriveRatios {
            // SDS MK4i - (8.14 : 1)
            public static final double L1 = (8.14 / 1.0);
            // SDS MK4i - (6.75 : 1)
            public static final double L2 = (6.75 / 1.0);
            // SDS MK4i - (6.12 : 1)
            public static final double L3 = (6.12 / 1.0);
        }

        public static final class MK4DriveRatios {
            // SDS MK4 - (8.14 : 1)
            public static final double L1 = (8.14 / 1.0);
            // SDS MK4 - (6.75 : 1)
            public static final double L2 = (6.75 / 1.0);
            // SDS MK4 - (6.12 : 1)
            public static final double L3 = (6.12 / 1.0);
            // SDS MK4 - (5.14 : 1)
            public static final double L4 = (5.14 / 1.0);
        }

        public static final class MK4iConstants {

            // Swerve module's wheel diameter
            public static final double wheelDiameter = 4;
            // The gear ratio of the steering motor
            public static final double angleGearRatio = ((150.0 / 7.0) / 1.0);

            // Whether or not, based on our module, the motors should be inverted
            public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
            public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;

            // Should the CANCoder's magnet be inverted as well?
            public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;

        }

        public static final class MK4Constants {
        
            // Swerve module's wheel diameter
            public static final double wheelDiameter = 4;

            // The gear ratio of the steering motor
            public static final double angleGearRatio = (12.8 / 1.0);

            // Whether or not, based on our module, the motors should be inverted
            public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
            public static final InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;

            // Should the CANCoder's magnet be inverted as well?
            public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;

        }

    }

    public static final class AlgaeIntake {
        public static final int INTAKEPIVOT_ID = 0;
        public static final int TOP_MOTOR_ID = 0;
        public static final int SENSOR_CHANNEL = 0;

        public static final double INTAKE_GEAR_RATIO = 0.0;
    }


    
    public static final double METERSTOINCHES = 39.3701;
}
