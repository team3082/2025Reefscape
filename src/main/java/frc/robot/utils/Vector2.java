package frc.robot.utils;

import frc.robot.Constants;

// import org.w3c.dom.views.DocumentView;

public class Vector2 {
    //The x an y doubles of the Vector2
    public double x;
    public double y; 
    
    /**
     * Constructor for a Vector2 with the coordinate at (0,0)
     */
    public Vector2() {
        x = 0;
        y = 0;
    }

    /**
     * Constructor for a Vector2 where you set the coordinate
     * @param x A double of the x of the Vector2
     * @param y A double of the y of the Vector2
     */
    public Vector2(double x, double y){
        this.x = x;
        this.y = y;
    }

    /**
     * Converts a polar vector to cartesian coordinates
     * @param theta A double of theta of the polar vector in radians
     * @param r A double of the magnitude of the polar vector
     * @return A Vector2 in cartesian coordinates
     */
    public static Vector2 fromPolar(double theta, double r){
        return new Vector2(Math.cos(theta) * r, Math.sin(theta) * r);
    }

    /**
    * Adds a Vector2 to this Vector2
    * @param vectorToAdd A Vector2 to add to the current one 
    * on the right hand side of the equation
    * @return A Vector2 that has been added
    */
    public Vector2 add(Vector2 vectorToAdd){
        return new Vector2(x + vectorToAdd.x, y + vectorToAdd.y);
    }

    /**
    * Subtracts a Vector2 to this Vector2
    * @param vectorToSub A Vector2 to subtract to the current one 
    * on the right hand side of the equation
    * @return A Vector2 that has been subtracted
    */
    public Vector2 sub(Vector2 vectorToSub){
        return new Vector2(x - vectorToSub.x, y - vectorToSub.y);
    }

    /**
    * Multiply this Vector2 by a scaler
    * @param scale A double of the scalar to multiply the Vector2 by
    * @return A Vector2 scaled by the value given
    */
    public Vector2 mul(double scale){
        return new Vector2(x * scale, y * scale);
    }

    /**
    * Divides this Vector2 by a scaler
    * @param scale A double of the scalar to divide the Vector2 by
    * @return A Vector2 divided by the value given
    */
    public Vector2 div(double scale){
        return new Vector2(x / scale, y / scale);
    }

    /**
    * Calculates the magnitude of this Vector2
    * @return A double of the magintude(also know as the hypotenuse)
    */
    public double mag(){
        return Math.hypot(x, y);
    }

    /**
    * Calculate a normalized(either -1, 0, or 1) version of this Vector2
    * @return A Vector2 that has been normalized
    */
    public Vector2 norm(){
        double m = mag();
        return div(m);
    }

    /**
    * Rotates the vector
    * @param angle A double of the angle in radians, by which to rotate the vector. Positive = counter clockwise
    * @return A Vector2 roated by angle
    */
    public Vector2 rotate(double angle){
        double rotateX = x * Math.cos(angle) - y * Math.sin(angle);
        double rotateY = x * Math.sin(angle) + y * Math.cos(angle);
        return new Vector2(rotateX, rotateY);
    }

    /**
     * Gets the dot product Vector2 of the Vector2
     * @param dotVector A Vector2 you want to dot by
     * @return A double of the dotproduct
     */
    public double dot(Vector2 dotVector){
        return x*dotVector.x + y*dotVector.y;
    }

    /**
    * Calculate the angle of the vector's direction using atan2
    * Circle begins at (1,0), positive rotating counter clockwise 
    * @return A double of the angle in radians 
    */
    public double atan2(){
        return Math.atan2(y, x);
    }

    /**
     * Sees if the current Vector2 is greater then another Vector2
     * @param otherVector The Vector2 to check if the current Vector2 is great then by
     * @return A boolean that is True if the current Vector2 is greater then otherVector
     */
    public boolean isGreater(Vector2 otherVector){
        return x > otherVector.x && y > otherVector.y;
    }
    
    /**
     * Sees the distance between this Vector2 and another Vector2
     * @param otherPoint A Vector2 of the other point
     * @return A double of the distance between this Vector2 and otherPoint
     */
    public double dist(Vector2 otherPoint){
        return this.sub(otherPoint).mag();
    }

    /**
     * Puts this Vector2 to a string
     */
    @Override
    public String toString(){
        return String.format("(%.2f, %.2f)", x, y);
    }

    /**
     * creates a copy of the vector
     * @return among us (sus) - good soup  POlson 2005 - 2025
     */
    public Vector2 copy() {
        return new Vector2(this.x, this.y);
    }

    public Vector2 convertToFieldCoords(){
        return new Vector2(this.x/Constants.METERSTOINCHES + 8.78,  this.y/Constants.METERSTOINCHES + 4.01);
    }
}