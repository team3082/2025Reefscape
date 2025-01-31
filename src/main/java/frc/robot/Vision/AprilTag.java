package frc.robot.vision;

import frc.robot.utils.Vector2;
import frc.robot.utils.Vector3;

public class AprilTag {

    private Vector3 pos;
    private double rotZ, rotY;

    /**
     * Represents an Apriltag on the field
     * @param x
     * @param y
     * @param z
     * @param rotZ Angle along the horizontal axis
     * @param rotY Angle along the vertical axis
     */
    public AprilTag(double x, double y, double z, double rotZ, double rotY) {

        this.pos = new Vector3(x, y, z);
        this.rotZ = rotZ;
        this.rotY = rotY;

    }

    /**
     * Returns the x and y field coordinates of the Apriltag
     * @return Vector2(x, y)
     */
    public Vector2 getPosition(){
        return new Vector2(pos.x, pos.y);
    }

    /**
     * Returns the x, y and z field coordinates of the Apriltag
     * @return Vector3(x, y, z)
     */
    public Vector3 get3DPosition(){
        return pos;
    }

    /**
     * Returns the angle of the Apriltag along the horizontal axis 
     * @return rotZ
     */
    public double getRotationZ(){
        return rotZ;
    }

    /**
     * Returns the angle of the Apriltag along the vertical axis 
     * @return rotY
     */
    public double getRotationY(){
        return rotY;
    }

}
