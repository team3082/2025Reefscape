package frc.robot.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.robot.Robot;

public class PIDController implements Sendable {

    private static int instances;
    
    protected double dest;

    double errorIntegral; // Used to calculate i
    double errorDerivative;
    double prevError; // Used to calculate d

    public double kP, kI, kD;
    public double deadband;
    public double velDeadband;

    public double max;

    protected long lastUpdate;

    // public double max, min;

    public PIDController(double p, double i, double d, double deadband, double velDeadband, double max){
        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.deadband = deadband;
        this.max = max;
        this.velDeadband = velDeadband;
        this.lastUpdate = RTime.absolute_frame() - 2;

        instances++;
        SendableRegistry.addLW(this, "PIDController", instances);
    }

    public double getP() {
        return this.kP;
    }

    public double getI() {
        return this.kI;
    }

    public double getD() {
        return this.kD;
    }

    public void setP(double p) {
        kP = p;
    }

    public void setI(double I) {
        kI = I;
    }

    public void setD(double D) {
        kD = D;
    }

    public double getDest() {
        return this.dest;
    }

    public void setDest(double dest){
        this.dest = dest;
        errorIntegral = 0;
        prevError = Double.MAX_VALUE;
    }

    /**
     * Updates the PIDController's state and returns its output.
     */
    public double updateOutput(double pos){
        // Update the timer for the last time the controller was updated 
        this.lastUpdate = RTime.absolute_frame();

        double output = 0.0;
        
        // Calculate proportional component based on error
        double error = dest - pos;
        output += kP * error;


        if (prevError == Double.MAX_VALUE) 
            prevError = error;

        // Calculate integral component with a trapezoidal Riemann sum
        errorIntegral += RTime.deltaTime() * (prevError + error) / 2;
        output += kI * errorIntegral;


        // Calculate derivative component based on mSec of the error
        if(Robot.isReal()){
            errorDerivative = (error - prevError) / RTime.deltaTime();
            output += kD * errorDerivative;
        }  
        prevError = error;
        
        // Clamp the output
        output = Math.max(Math.min(output, max), -max);


        return output;
    }

    /**
     * Checks whether this PIDController is active by testing whether it was updated during the previous frame.
     */
    public boolean isActive() {
        return RTime.absolute_frame() <= this.lastUpdate + 1;
    }

    public boolean atSetpoint(){
        return Math.abs(prevError) < deadband && Math.abs(errorDerivative) < velDeadband;
    }
    
    public double getError() {
        return this.prevError;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("P", this::getP, this::setP);
        builder.addDoubleProperty("I", this::getI, this::setI);
        builder.addDoubleProperty("D", this::getD, this::setD);
        builder.addDoubleProperty("Setpoint", this::getDest, this::setDest);
    }

}