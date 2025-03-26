package frc.robot.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.utils.Vector2;

public class Camera {

    public PhotonCamera photonCamera;
    public Vector2 robotToCamera;
    public double cameraPitch;
    public double cameraYaw;
    public PhotonTrackedTarget latestTarget;
    private boolean disabled;

    public Camera(PhotonCamera photonCamera, Vector2 robotToCamera, double cameraPitch, double cameraYaw) {
        this.photonCamera = photonCamera;
        this.robotToCamera = robotToCamera;
        this.cameraPitch = cameraPitch;
        this.cameraYaw = cameraYaw;
        disabled = false;
    }

    public boolean isLatestTarget(PhotonTrackedTarget target) {
        return (target.equals(latestTarget));
    }

    public void setLatestTarget(PhotonTrackedTarget target) {
        this.latestTarget = target;
    }

    public void disable(){
        disabled = true;
    }
    public void enable(){
        disabled = false;
    }

    public boolean isDisabled(){
        return disabled;
    }

}
