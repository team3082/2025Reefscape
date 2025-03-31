package frc.robot.utils.ChickenPlanner;

import frc.robot.utils.trajectories.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Represents a trajectory consisting of a sequence of paths and associated action points.
 * 
 * @param paths the list of paths in the trajectory.
 * @param actionPoints the list of action points (e.g., rotations, stops) in the trajectory.
 */
public record ChickenTrajectory(List<Path> paths, List<ActionPoint> actionPoints, boolean commandAtEndPoint) {}
