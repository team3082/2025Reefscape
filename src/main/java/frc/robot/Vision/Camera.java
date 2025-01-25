package frc.robot.vision;

import org.photonvision.PhotonCamera;

import frc.robot.utils.Vector2;

public class Camera {

    public PhotonCamera photonCamera;
    public Vector2 robotToCamera;
    public double cameraPitch;
    public double cameraYaw;

    public Camera(PhotonCamera photonCamera, Vector2 robotToCamera, double cameraPitch, double cameraYaw) {

        this.photonCamera = photonCamera;
        this.robotToCamera = robotToCamera;
        this.cameraPitch = cameraPitch;
        this.cameraYaw = cameraYaw;

    }
}