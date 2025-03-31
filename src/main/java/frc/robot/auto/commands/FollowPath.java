package frc.robot.auto.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.PIDController;
import frc.robot.utils.Vector2;
import frc.robot.utils.trajectories.*;

/**
 * Command to follow a specified trajectory path using PID controllers for both movement and rotation.
 */
public class FollowPath extends Command {

    // Path used for following the trajectory
    private ArrayList<Path> futurePaths;
    private Path currentPath;
    private ArrayList<Path> pastPaths;

    // PID Controllers for movement and rotation
    private PIDController movementPID = new PIDController(3.0, 0.025, 0.2, 0.01, 0.00, 1.0);
    private PIDController rotationPID = new PIDController(0.75, 0.0, 0.15, 0.01, 0.0, 0.5);

    // Variables to track the current state of the path following
    private double currentT;
    private boolean isFinished = false;

    /**
     * Constructor to initialize FollowPath with a single path.
     */
    public FollowPath(Path path, ArrayList<Path> past, ArrayList<Path> future) {
        this.futurePaths = future;
        this.currentPath = path;
        this.pastPaths = past;
    }

    /**
     * Calculates the total length traveled along the path.
     */
    private double getPathLengthTraveled() {
        double lengthTravelled = 0;
        for(Path path : pastPaths){
            lengthTravelled += path.getTotalLength();
        }
        return lengthTravelled + currentPath.computeRemainingLength(currentT);
    }

    /**
     * Calculates the total length of the path.
     */
    private double getTotalPathLength() {
        double totalLength = 0;
        for(Path path : pastPaths){
            totalLength += path.getTotalLength();
        }
        for(Path path : futurePaths){
            totalLength += path.getTotalLength();
        }
        return totalLength + currentPath.getTotalLength();
    }

    @Override
    public void initialize() {
        currentT = currentPath.findClosestT(SwervePosition.getPosition());

        movementPID.setDest(1);
        rotationPID.setDest(currentPath.getFirstPoint().getRotation());
    }

    /**
     * Executes the path following logic, adjusting the robot's movement and rotation.
     */
    @Override
    public void execute() {
        double currentRotation = Pigeon.getRotationRad() % (2.0 * Math.PI);
        currentT = currentPath.findClosestT(SwervePosition.getPosition());

        double percentTraveled = (getTotalPathLength() - getPathLengthTraveled()) / getTotalPathLength();
        double movementOutput = movementPID.updateOutput(percentTraveled);

        Point targetPoint = currentPath.getNearestFollowPoint(currentT, 0.03);
        Vector2 targetVector = targetPoint.getPosition();
        Vector2 errorVector = targetVector.sub(SwervePosition.getPosition()).rotate(-Robot.getAllianceMultiplier() * Math.PI/2);

        SwerveManager.rotateAndDrive(0, errorVector.norm().mul(movementOutput));
        
        isFinished = targetPoint.getPosition().sub(SwervePosition.getPosition()).mag() < Tuning.CURVE_DEADBAND;
    }

    /**
     * Ends the command and stops the robot.
     * Stops the swerve modules both in real and simulation modes.
     */
    @Override
    public void end(boolean interrupted) {
        if(!interrupted && futurePaths.size()>0) return;
        SwerveManager.rotateAndDrive(0, new Vector2());
        SwerveManager.update();

        // If in simulation, stop the swerve modules
        if (Robot.isSimulation()) {
            for (int i = 0; i < SwerveManager.mods.length; i++) {
                SwerveManager.mods[i].simModule.speed = 0;
            }
        }
    }

    /**
     * Returns whether the command is finished.
     */
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
