package frc.robot.swerve;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.utils.Vector2;
import frc.robot.utils.swerve.SwerveMath;

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

                    double position = mods[i].getDrivePosition();
                    double disp = position - previousDrivePositions[i];
                    previousDrivePositions[i] = position;

                    double angle = mods[i].getSteerAngle();

                    robotDisp = robotDisp.add(Vector2.fromPolar(angle, disp));
                }

                Vector2 meanDisp = robotDisp.div(mods.length);

                double pigeonAngle = Pigeon.getRotationRad();
                double deltaAngle = 0.0;
                if(previousPigeonAngle != Double.NaN){
                    deltaAngle = pigeonAngle - previousPigeonAngle;
                }
                
                //lil fix
                Vector2 innovation = SwerveMath.poseExponentiation(meanDisp, previousPigeonAngle - Math.PI/2.0, deltaAngle);
                
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
}
