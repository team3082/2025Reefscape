package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.ChickenPlanner.ActionPoint;
import frc.robot.utils.ChickenPlanner.ChickenParser;
import frc.robot.utils.ChickenPlanner.ChickenTrajectory;
import frc.robot.utils.trajectories.Path;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * Class to follow a trajectory routine, executing specified actions at defined points.
 */
public class FollowChickenRoutine extends SequentialCommandGroup {

    /**
     * Constructor to initialize the FollowChickenRoutine.
     * @param trajectoryName The name of the trajectory.
     * @param actionPointCommands The array of commands to execute at specific action points.
     */
    public FollowChickenRoutine(String trajectoryName, boolean resetPosition, Command... actionPointCommands) {
        ChickenTrajectory chickenTrajectory;

        try {
            chickenTrajectory = ChickenParser.getTrajectory(trajectoryName+".json");
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        List<ActionPoint> actionPoints = chickenTrajectory.actionPoints();
        List<Path> paths = chickenTrajectory.paths();

        if(resetPosition) addCommands(Commands.runOnce(() -> SwervePosition.setPosition(paths.get(0).getFirstPoint().getPosition())));
       
        try{
            for (int index = 0; index < actionPoints.size(); index++) {
                ActionPoint currentActionPoint = actionPoints.get(index);

                // If the action point is stopped
                if (currentActionPoint.isStopped()) {
                    addCommands(new FollowPath(paths.get(index), new ArrayList<>(), new ArrayList<>()));  // Follow the current path
                    if (actionPointCommands.length > index) {
                        addCommands(actionPointCommands[index]);  // Execute the action point command if available
                        
                    } else {
                        int localIndex = index;
                        addCommands(Commands.runOnce(() -> System.out.println("Stopped at action point: " + localIndex)));
                    }

                    if(index == actionPoints.size()-1){
                        addCommands(new FollowPath(paths.get(index+1), new ArrayList<>(), new ArrayList<>()));
                    }
                    continue;
                }

                // If the action point is non-stopping, combine the current and next path
                if (index + 1 < paths.size()) {
                    int localIndex = index;

                    ArrayList<Path> pastPaths = new ArrayList<>();
                    int innerIndex = index-1;
                    while(actionPoints.get(innerIndex).isStopped()){
                        pastPaths.add(0, paths.get(index));
                    }

                    ArrayList<Path> futurePaths = new ArrayList<>();
                    innerIndex = index+1;
                    while(actionPoints.get(innerIndex).isStopped() && index != actionPointCommands.length){
                        futurePaths.add(paths.get(index));
                    }

                    addCommands(new FollowPath(paths.get(index), pastPaths, futurePaths));
                }
            }

            if(chickenTrajectory.commandAtEndPoint()){
                if (actionPointCommands.length > actionPoints.size()) {
                    addCommands(actionPointCommands[actionPoints.size()]);
                } else {
                    addCommands(Commands.runOnce(() -> System.out.println("Stopped at action point: " + actionPoints.size())));
                }

            }
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }
    }
}
