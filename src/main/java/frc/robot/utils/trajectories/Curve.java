package frc.robot.utils.trajectories;

import java.util.List;

import frc.robot.utils.Vector2;

public interface Curve {
    void getEvenlySpacedPoints();
    Vector2 getPoint(double t);
    List<Vector2> getPoints();
    Curve reverse();
    Curve flipHorizontal();
    Curve rotate(double angle);
}
