package frc.robot.utils.swerve;

import frc.robot.utils.Vector2;

public class SwerveMath {

    /**
     * returns an estimate of innovation using pose exponentiation
     * @param deltaPos the linear displacement of the system wrt the robot
     * @param theta0 the initial angle of the velocity
     * @param deltaTheta the change in theta throughout the timestep
     * @return a Vector2 object containing the displacement of the system after the timestep assuming constant drive and steer velocities
     */
    public static Vector2 poseExponentiation(double deltaPos, double theta0, double deltaTheta){
        if(deltaTheta == 0.0){
            return Vector2.fromPolar(theta0, deltaPos);
        }

        double deltax = (Math.sin(theta0 + deltaTheta) - Math.sin(theta0)) / deltaTheta * deltaPos;
        double deltay = -(Math.cos(theta0 + deltaTheta) - Math.cos(theta0)) / deltaTheta * deltaPos;

        return new Vector2(deltax, deltay);

    }

    public static Vector2 poseExponentiation(Vector2 deltaPos, double theta0, double deltaTheta){
        return poseExponentiation(deltaPos.mag(), theta0 + deltaPos.atan2(), deltaTheta);
    }

}