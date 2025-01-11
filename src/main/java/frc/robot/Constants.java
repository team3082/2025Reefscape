package frc.robot;

import frc.robot.vision.AprilTag;

public class Constants {
    
    public final AprilTag[] APRIL_TAGS = {
        null,
        // Red Alliance
        new AprilTag(311.93, -132.7, 58.5, Math.toRadians(126), 0), // id 1
        new AprilTag(311.93, 132.7, 58.5, Math.toRadians(234), 0), // id 2
        new AprilTag(109.71, 158.65, 51.25, Math.toRadians(270), 0), // id 3
        new AprilTag(19.76, 83.14, 73.54, 0, Math.toRadians(30)), // id 4
        new AprilTag(19.76, -83.11, 73.54, 0, Math.toRadians(30)), // id 5
        new AprilTag(185.05, -28.33, 12.13, Math.toRadians(300), 0), // id 6
        new AprilTag(201.43, 0, 12.13, Math.toRadians(0), 0), // id 7
        new AprilTag(185.05, 28.33, 12.13, Math.toRadians(60), 0), // id 8
        new AprilTag(152.33, 28.33, 12.13, Math.toRadians(120), 0), // id 9
        new AprilTag(135.95, 0, 12.13, Math.toRadians(180), 0), // id 10
        new AprilTag(152.33, -28.33, 12.13, Math.toRadians(240), 0), // id 11

        // Blue Alliance
        new AprilTag(-311.93, -132.7, 58.5, Math.toRadians(54), 0), // id 12
        new AprilTag(-311.93, 132.7, 58.5, Math.toRadians(306), 0), // id 13
        new AprilTag(-19.76, 83.14, 73.54, Math.toRadians(180), Math.toRadians(30)), // id 14
        new AprilTag(-19.76, -83.11, 73.54, Math.toRadians(180), Math.toRadians(30)), // id 15
        new AprilTag(-109.71, -158.65, 51.25, Math.toRadians(90), 0), // id 16
        new AprilTag(-185.05, -28.33, 12.13, Math.toRadians(240), 0), // id 17
        new AprilTag(-201.43, 0, 12.13, Math.toRadians(180), 0), // id 18
        new AprilTag(-185.05, 28.33, 12.13, Math.toRadians(120), 0), // id 19
        new AprilTag(-152.33, 28.33, 12.13, Math.toRadians(60), 0), // id 20
        new AprilTag(-135.95, 0, 12.13, Math.toRadians(0), 0), // id 21
        new AprilTag(-152.33, -28.33, 12.13, Math.toRadians(300), 0), // id 22
        
    };
}
