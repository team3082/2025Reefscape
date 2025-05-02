package frc.robot.auto.commands;

import frc.robot.Tuning;
import frc.robot.utils.Vector2;
import frc.robot.utils.PIDController;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.trajectories.Curve;
import frc.robot.subsystems.sensors.Pigeon;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.trajectories.RobotPath;

public class FollowCurveStraight extends Command {
    private Vector2 endPos;
    private double startDistance;
    private RobotPath path;
    private boolean isFinished = false;

    private PIDController movePID;
    private PIDController rotPID;

    double maxSpeed = 1.0;

    public FollowCurveStraight(Curve curve, double targetRot, double maxVelMove, double maxVelRot) {
        this.path = new RobotPath(curve.getPoints(), targetRot);
        this.endPos = path.getLastPos();
        System.out.println("EndPos: " + endPos);
        this.movePID = new PIDController(1.75, 0.025, 0.1575, 0.0, 0.0, maxVelMove);
        this.rotPID = new PIDController(0.35, 0.015, 0.08, 0.035, 0.1, maxVelMove);
    }

    @Override
    public void initialize() {
        System.out.println("FollowCurve Initialized");
        this.movePID.setDest(1);
        this.rotPID.setDest((path.getTargetRot() + Math.PI / 2.0) % (2.0 * Math.PI));
        this.startDistance = SwervePosition.getPosition().sub(endPos).mag();
    }

    @Override
    public void execute() {
        path.updatePosition(SwervePosition.getPosition());
        double currentRot = Pigeon.getRotationRad() % (2.0 * Math.PI);
        if (currentRot - rotPID.getDest() > Math.PI) {
            currentRot -= 2.0 * Math.PI;
        } else if (currentRot - rotPID.getDest() < -Math.PI) {
            currentRot += 2.0 * Math.PI;
        }

        double moveOutput = movePID.updateOutput((startDistance - SwervePosition.getPosition().sub(endPos).mag()) / startDistance);
        double rotOutput = rotPID.updateOutput(currentRot);

        Vector2 driveVector = endPos.sub(SwervePosition.getPosition()).norm().mul(moveOutput);
        System.out.println("Drive Vector: " + driveVector);

        // Vector2 driveVector = path.getDriveVector().norm().mul(moveOutput);
        driveVector = driveVector.rotate(-Math.PI / 2.0);

        if (driveVector.mag() > maxSpeed) {
            driveVector = driveVector.norm().mul(maxSpeed);
        }

        SwerveManager.rotateAndDrive(rotOutput, driveVector);

        boolean driveFinished = (SwervePosition.getPosition().sub(path.getLastPos()).mag() < Tuning.CURVE_DEADBAND);
        if (driveFinished) driveVector = new Vector2();
        boolean rotFinished = rotPID.atSetpoint();
        if (rotFinished) rotOutput = 0.0;

        if (driveFinished && rotFinished) {
            isFinished = true;
            end(false);
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        SwerveManager.rotateAndDrive(0.0, new Vector2());
        System.out.println("FollowCurve ended");
    }

    public double getRemainingPathLength() {
        return path.getRemainingPathLength();
    }

    public void setMaxSpeed(double speed) {
        this.maxSpeed = speed;
    }
}
