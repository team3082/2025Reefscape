package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.utils.Vector2;
import frc.robot.vision.VisionManager;

import static frc.robot.swerve.SwerveManager.mods;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

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
            System.out.println("Updating odom");
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

                position = position.add(innovation);

                Optional<Vector2> visionPos = VisionManager.getPosition(pigeonAngle);
                

                // TODO Migrate to Telemetry
                try{
                    if (!visionPos.isEmpty() && VisionManager.isEnabled()) {
                        // Pose2d visionPose = new Pose2d(visionPos.get().rotate(Math.PI/2.0).x/Constants.METERSTOINCHES + 8.78, 
                        //                             visionPos.get().rotate(Math.PI/2.0).y/Constants.METERSTOINCHES + 4.01, 
                        //                             Rotation2d.fromRadians(Pigeon.getRotationRad()+ Robot.getAllianceMultiplier() * Math.PI / 2.0));
                        // Logger.recordOutput("Robot/Vision/Vision Pose", visionPose);
                        // Logger.recordOutput("Robot/Vision/Position", visionPos.get().rotate(Math.PI/2).toString());
                        // Logger.recordOutput("Robot/Vision/Position/x", visionPos.get().rotate(Math.PI/2).x);
                        // Logger.recordOutput("Robot/Vision/Position/y", visionPos.get().rotate(Math.PI/2).y);
                        position = new Vector2(-visionPos.get().y, visionPos.get().x);
                    } 
                } catch (Exception e) {}
                try {
                    sleep(7);
                } catch (InterruptedException e) {
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

    public static Vector2 poseExponentiation(Vector2 deltaPos, double theta0, double deltaTheta){
        return poseExponentiation(deltaPos.mag(), theta0 + deltaPos.atan2(), deltaTheta).rotate(/*(DriverStation.getAlliance().get() == Alliance.Blue ? -1 : 1) */ Math.PI/2);
    }

    public static void setPosition(Vector2 pos){
        synchronized(positionLock){
            position = pos;
        }
    }

}
