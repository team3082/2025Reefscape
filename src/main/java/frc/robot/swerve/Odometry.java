package frc.robot.swerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.utils.Vector2;

import static frc.robot.swerve.SwerveManager.mods;

public class Odometry {

    /**
     * this is the position of the robot relative to an arbitrary position of (0,0)
     * if you need innovation you will need to do that on the calling end
     */
    private static Vector2 position;
    //a lock to make sure that position isn't being retrieved and updated at the same time
    private static Object positionLock = new Object();

    private static double lastLoopTimeStamp;

    private static double[] previousDrivePositions = new double[mods.length];
    private static double previousPigeonAngle;

    public static void init(){
        lastLoopTimeStamp = Timer.getFPGATimestamp();
        position = new Vector2(0.0,0.0);
        previousPigeonAngle = Pigeon.getRotationRad();

        odomThread.setDaemon(true);
        odomThread.start();
    }
    
    private static Thread odomThread= new Thread(){
        @Override
        public void run(){
            while(!isInterrupted()){
                double deltaTime = Timer.getFPGATimestamp() - lastLoopTimeStamp;
                lastLoopTimeStamp += deltaTime;

                //how far the robot has moved in robot frame since the last loop
                Vector2 robotDisp = new Vector2();
                
                for(int i = 0; i < mods.length; i++){

                    double drivePosition = mods[i].getDrivePosition();
                    double disp = drivePosition - previousDrivePositions[i];
                    previousDrivePositions[i] = drivePosition;

                    double angle = mods[i].getSteerAngle();

                    robotDisp = robotDisp.add(Vector2.fromPolar(angle, disp));
                }

                Vector2 meanDisp = robotDisp.div(mods.length);

                double pigeonAngle = Pigeon.getRotationRad();
                double deltaAngle = 0.0;
                if(previousPigeonAngle != Double.NaN){
                    deltaAngle = pigeonAngle - previousPigeonAngle;
                }
                
                //- Math.PI/2.0 is becuase pigeon rotation is offset
                Vector2 innovation = poseExponentiation(meanDisp, previousPigeonAngle - Math.PI/2, deltaAngle);
                previousPigeonAngle = pigeonAngle;

                synchronized(positionLock){
                    position = position.add(innovation);
                }
                
                try {
                    sleep(7);
                } catch (InterruptedException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
        }
    };

    public static Vector2 getPosition(){
        synchronized(positionLock){
            return position;
        }
    }

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

    // TODO Merge into one method
    public static Vector2 poseExponentiation(Vector2 deltaPos, double theta0, double deltaTheta){
        return poseExponentiation(deltaPos.mag(), theta0 + deltaPos.atan2(), deltaTheta).rotate((DriverStation.getAlliance().get() == Alliance.Blue ? -1 : 1) * Math.PI/2);
    }

}
