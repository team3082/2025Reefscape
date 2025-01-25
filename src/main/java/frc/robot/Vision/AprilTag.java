package frc.robot.vision;

import frc.robot.utils.Vector3;

public class AprilTag {

    public Vector3 pos;
    public double x, y, z, rotZ, rotY;

    public AprilTag(double x, double y, double z, double rotZ, double rotY) {

        this.pos = new Vector3(x, y, z);
        this.rotZ = rotZ;
        this.rotY = rotY;

    }

}
