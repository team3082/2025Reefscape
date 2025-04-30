package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Tuning;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.PIDController;
import frc.robot.utils.RTime;
import frc.robot.utils.TrapezoidalProfile;
import frc.robot.utils.Vector2;
import frc.robot.utils.trajectories.Curve;
import frc.robot.utils.trajectories.RobotPath;

public class FollowCurveTrapezoidal extends Command {
    private RobotPath path;
    private TrapezoidalProfile moveProfile;
    private PIDController rotPID;

    private double startTime;

    private boolean isFinished = false;

    public FollowCurveTrapezoidal(Curve curve, double targetRot, TrapezoidalProfile moveProfile, double maxVelRot) {
        // Initialize the curve and profile
        this.path = new RobotPath(curve.getPoints(), targetRot);
        this.moveProfile = moveProfile;
        this.rotPID = new PIDController(0.35, 0.015, 0.08, 0.035, 0.1, maxVelRot);
        this.startTime = RTime.now();
    }

    @Override
    public void initialize() {
        // Initialize the command
        System.out.println("FollowCurveTrapezoidal Initialized");
        System.out.println("path length: " + path.getPathLength());
        moveProfile.generateProfile(path.getPathLength());
        this.rotPID.setDest((path.getTargetRot() + Math.PI / 2.0) % (2.0 * Math.PI));
    }

    @Override
    public void execute() {
        path.updatePosition(SwervePosition.getPosition());
        Vector2 driveVector = path.getDriveVector().norm().rotate(-Math.PI / 2.0);
        double driveOutput = moveProfile.getVelocity(path.getPathLength() - path.getRemainingPathLength()) / Constants.Swerve.PERCENT_OUT_TO_MOVE_VEL;
        driveVector = driveVector.mul(driveOutput + 0.05);

        double currentRot = Pigeon.getRotationRad() % (2.0 * Math.PI);
        if (currentRot - rotPID.getDest() > Math.PI) {
            currentRot -= 2.0 * Math.PI;
        } else if (currentRot - rotPID.getDest() < -Math.PI) {
            currentRot += 2.0 * Math.PI;
        }
        double rotOutput = rotPID.updateOutput(currentRot);

        // System.out.println("Drive Vector: " + driveVector + " rotOutput: " + rotOutput);
        System.out.println("DistanceL " + (path.getPathLength() - path.getRemainingPathLength()) + " Drive Vel: " + moveProfile.getVelocity(path.getPathLength() - path.getRemainingPathLength()));

        boolean driveFinished = false;
        boolean rotFinished = false;

        if (path.getRemainingPathLength() < Tuning.CURVE_DEADBAND) {
            driveVector = new Vector2();
            driveFinished = true;
        }
        if (rotPID.atSetpoint()) {
            rotOutput = 0;
            rotFinished = true;
        }

        if (rotFinished && driveFinished) {
            isFinished = true;
            end(false);
        } else {
            SwerveManager.rotateAndDrive(rotOutput, driveVector);
        }
    }

    @Override
    public boolean isFinished() {
        // Check if the command is finished
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        // End the command
        System.out.println("FollowCurveTrapezoidal Ended");
        SwerveManager.rotateAndDrive(0, new Vector2());
    }

    public double getRemainingPathLength() {
        return path.getRemainingPathLength();
    }
}
