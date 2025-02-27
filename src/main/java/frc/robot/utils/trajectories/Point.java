package frc.robot.utils.trajectories;

import frc.robot.utils.Vector2;

/**
 * Represents a point in a path with position, rotation, and t-value.
 */
public class Point {
        private final Vector2 position;
        private double rotation;
        private final double t;
    
        /**
         * Constructs a new Point object.
         *
         * @param position The position vector of the point.
         * @param rotation The rotation angle at this point.
         * @param t The normalized t-value representing the point's position on the path.
         */
        public Point(Vector2 position, double rotation, double t) {
            this.position = new Vector2(position);
            this.rotation = rotation;
            this.t = t;
        }
    
        /**
         * Gets the position of the point.
         *
         * @return A copy of the position vector.
         */
        public Vector2 getPosition() {
            return new Vector2(position);
        }
    
        /**
         * Gets the rotation at this point.
         *
         * @return The rotation as an Angle instance.
         */
        public double getRotation() {
            return rotation;
        }
    
        /**
         * Sets the rotation of the point
         * @param rotation The new rotation of the point
         */
        public void setRotation(double rotation){
            this.rotation = rotation;
    }

    /**
     * Gets the t-value of this point.
     *
     * @return The normalized t-value.
     */
    public double getT() {
        return t;
    }
}
