package frc.robot.vision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.REEF_POSITIONS;
import frc.robot.utils.Vector2;
import frc.robot.utils.Vector3;

public class AprilTag {

    private Vector3 pos;
    private double rotZ, rotY;
    private int id;

    /**
     * Represents an Apriltag on the field
     * @param x
     * @param y
     * @param z
     * @param rotZ Angle along the horizontal axis
     * @param rotY Angle along the vertical axis
     */
    public AprilTag(int id, double x, double y, double z, double rotZ, double rotY) {

        this.id = id;
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
     * returns the id of the apriltag
     * @return the id of this tag
     */
    public int getID() {
        return id;
    }

    /**
     * Gets the position for the front of the robot to be at the apriltag position
     * @return the position of the robot if it were right in front of the apriltag
     */
    public Vector2 getCenterPosition() {
        return getPosition().add(new Vector2(Constants.Swerve.WIDTH/2 + 4, 0).rotate(getRotationZ()));
    }

    /**
     * Gets the robot position in front of the left pole of the reef.
     * This should only be used if the apriltag is at the reef
     * @return the robot position in front of the left pole of the face of the reef
     */
    public Vector2 getLeftPosition() {
        Vector2 individualPoleOffset = DriverStation.getAlliance().get() == Alliance.Red ? REEF_POSITIONS.getPositionFromTagID(id, false).redOffset : REEF_POSITIONS.getPositionFromTagID(id, false).blueOffset;
        Vector2 alignOffset = new Vector2(6.5, 0.0).add(individualPoleOffset).rotate(getRotationZ() - Math.PI / 2);
        return getCenterPosition().add(alignOffset);
    }

    /**
     * Gets the robot position in front of the right pole of the reef.
     * This should only be used if the apriltag is at the reef
     * @return the robot position in front of the right pole of the face of the reef
     */
    public Vector2 getRightPosition() {
        Vector2 individualPoleOffset = DriverStation.getAlliance().get() == Alliance.Red ? REEF_POSITIONS.getPositionFromTagID(id, true).redOffset : REEF_POSITIONS.getPositionFromTagID(id, true).blueOffset;
        Vector2 alignOffset = new Vector2(6.5, 0.0).add(individualPoleOffset).rotate(getRotationZ() + Math.PI / 2);
        return getCenterPosition().add(alignOffset);
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
