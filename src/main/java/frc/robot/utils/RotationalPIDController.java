package frc.robot.utils;

public class RotationalPIDController extends PIDController{

    public RotationalPIDController(double p, double i, double d, double deadband, double velDeadband, double maxOutput){
        super(p,i,d,deadband,velDeadband,maxOutput);
    }

    @Override
    public void setDest(double dest) {
        super.setDest(RMath.modulo(dest, 2.0 * Math.PI));
    }

    //woohoo it works (trust)
    @Override
    public double updateOutput(double pos){
        pos = RMath.modulo(pos, 2.0 * Math.PI);

        if(pos - dest > Math.PI){
            pos -= 2.0 * Math.PI;
        }else if(dest - pos > Math.PI){
            pos += 2.0 * Math.PI;
        }
        
        return super.updateOutput(pos);
    }

}
