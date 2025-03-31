package frc.robot.utils.ChickenPlanner;


/**
 * A record representing am action point along a trajectory.
 * 
 * @param t The t value at which this action occurs.
 * @param isStopped Whether the action represents a stop (true) or a non-stop action (false).
 * @param rotation The desired robot rotation at this action point.
 */
public record ActionPoint(double t, boolean isStopped, double rotation) {}
