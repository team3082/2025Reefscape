package frc.robot.utils;

import frc.robot.Tuning;

public class TrapezoidalProfile {
    private double maxVelocity;
    private double maxAcceleration;
    private double maxDeceleration;
    private double[][] profile; // Stores distance and velocity for each point

    public enum TrapezoidalPreset {
        MOVE_PRECISE,
        MOVE_FAST,
        ROT_PRECISE,
        ROT_FAST
    }

    public TrapezoidalProfile(double maxVelocity, double maxAcceleration, double maxDeceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.profile = new double[100][2]; // Initialize profile with distance and velocity
    }

    public TrapezoidalProfile(TrapezoidalPreset preset) {
        switch (preset) {
            case MOVE_PRECISE:
                this.maxVelocity = Tuning.MOVE_PRECISE_VEL; 
                this.maxAcceleration = Tuning.MOVE_PRECISE_ACC; 
                this.maxDeceleration = Tuning.MOVE_PRECISE_DEC; 
                break;

            case MOVE_FAST:
                this.maxVelocity = Tuning.MOVE_FAST_VEL; 
                this.maxAcceleration = Tuning.MOVE_FAST_ACC; 
                this.maxDeceleration = Tuning.MOVE_FAST_DEC; 
                break;

            case ROT_PRECISE:
                this.maxVelocity = Tuning.ROT_PRECISE_VEL;
                this.maxAcceleration = Tuning.ROT_PRECISE_ACC;
                this.maxDeceleration = Tuning.ROT_PRECISE_DEC;
                break;

            case ROT_FAST:
                this.maxVelocity = Tuning.ROT_FAST_VEL;
                this.maxAcceleration = Tuning.ROT_FAST_ACC;
                this.maxDeceleration = Tuning.ROT_FAST_DEC;
        }
        this.profile = new double[100][2]; // Initialize profile with distance and velocity
    }

    public void generateProfile(double totalDistance) {
        double timeStep = 0.02; // 20ms time step
        double currentVelocity = 0;
        double currentPosition = 0;

        // Calculate total time for each phase
        double accelTime = maxVelocity / maxAcceleration;
        double accelDistance = 0.5 * maxAcceleration * accelTime * accelTime;

        double decelTime = maxVelocity / maxDeceleration;
        double decelDistance = 0.5 * maxDeceleration * decelTime * decelTime;

        double cruiseDistance = totalDistance - (accelDistance + decelDistance);
        double cruiseTime = cruiseDistance / maxVelocity;

        if (cruiseDistance < 0) {
            // Adjust for triangular profile (no cruise phase)
            accelTime = Math.sqrt(totalDistance / (0.5 * maxAcceleration + 0.5 * maxDeceleration));
            accelDistance = 0.5 * maxAcceleration * accelTime * accelTime;
            decelTime = accelTime;
            decelDistance = totalDistance - accelDistance;
            cruiseTime = 0;
        }

        double totalTime = accelTime + cruiseTime + decelTime;
        int totalSteps = (int) Math.ceil(totalTime / timeStep);

        profile = new double[totalSteps][2]; // Resize profile array to fit all steps
        int index = 0;

        // Acceleration phase
        while (index < totalSteps && currentPosition < accelDistance) {
            currentVelocity = Math.min(currentVelocity + maxAcceleration * timeStep, maxVelocity);
            currentPosition += currentVelocity * timeStep;
            profile[index][0] = currentPosition;
            profile[index][1] = currentVelocity;
            index++;
        }

        // Constant velocity phase
        while (index < totalSteps && currentPosition < accelDistance + cruiseDistance) {
            currentPosition += maxVelocity * timeStep;
            profile[index][0] = currentPosition;
            profile[index][1] = maxVelocity;
            index++;
        }

        // Deceleration phase
        while (index < totalSteps && currentPosition < totalDistance) {
            currentVelocity = Math.max(currentVelocity - maxDeceleration * timeStep, 0);
            currentPosition += currentVelocity * timeStep;
            profile[index][0] = currentPosition;
            profile[index][1] = currentVelocity;
            index++;
        }

        // Fill remaining profile with zeros if necessary
        while (index < totalSteps) {
            profile[index][0] = totalDistance;
            profile[index][1] = 0;
            index++;
        }

        // Print the profile
        printProfile();
    }

    public void printProfile() {
        System.out.println("Generated Trapezoidal Profile:");
        for (int i = 0; i < profile.length; i++) {
            System.out.printf("Step %d: Position = %.2f, Velocity = %.2f%n", i, profile[i][0], profile[i][1]);
        }
    }

    public double getVelocity(double distance) {
        for (int i = 0; i < profile.length - 1; i++) {
            if (profile[i][0] <= distance && profile[i + 1][0] > distance) {
                return profile[i][1];
            }
        }
        return 0; // Return 0 if distance is out of bounds
    }
}
