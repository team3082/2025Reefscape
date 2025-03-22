package frc.robot.swerve;

import frc.robot.utils.Vector2;

public class OdometryBuffer {
    private final int BUFFER_SIZE = 2;
    private Vector2[] buffer = new Vector2[BUFFER_SIZE];

    public OdometryBuffer() {}

    public void addValue(Vector2 newValue) {
        // System.out.println("adding value trust me bro");
        Vector2[] initialBuffer = this.buffer.clone();

        buffer[0] = newValue;
        for (int i = 1; i < BUFFER_SIZE; i++) {
            buffer[i] = initialBuffer[i - 1];
        }

    }

    public Vector2 getTotalBuffer() {
        Vector2 totalBuffer = new Vector2();
        for (Vector2 vector : buffer) {
            totalBuffer = totalBuffer.add(vector);
        }
        return totalBuffer;
    }
}