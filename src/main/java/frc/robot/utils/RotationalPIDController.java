package frc.robot.utils;

public class RotationalPIDController extends PIDController{
    public final double minValue;
    public final double maxValue;
    public final double range;

    public RotationalPIDController(double p, double i, double d, double deadband, double velDeadband, double maxOutput, double minValue, double maxValue){
        super(p,i,d,deadband,velDeadband,maxOutput);
        this.minValue = minValue;
        this.maxValue = maxValue;
        range = maxValue - minValue;
    }

    public RotationalPIDController(double p, double i, double d, double deadband, double velDeadband, double maxOutput){
        super(p,i,d,deadband,velDeadband,maxOutput);
        this.minValue = -Math.PI;
        this.maxValue = Math.PI;
        range = Math.PI*2;
    }

    @Override
    public void setDest(double dest){
        dest = RMath.modulo(dest - minValue, range) + minValue;
    }

    @Override
    public double updateOutput(double pos){
        pos = RMath.modulo(pos - minValue, range) + minValue;
        double diff = dest - pos;
        if(diff > range / 2){
            pos += range;
        }else if(diff < -range / 2){
            pos -= range;
        }
        return super.updateOutput(pos);
    }

    
}