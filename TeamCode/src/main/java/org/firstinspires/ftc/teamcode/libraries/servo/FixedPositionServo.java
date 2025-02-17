package org.firstinspires.ftc.teamcode.libraries.servo;

// a fixed set position servo class
public class FixedPositionServo extends Servob {
    public final String servoName;
    private final double[] positions;

    public FixedPositionServo(String servoName, double[] positions){
        super(servoName);
        this.servoName = servoName;
        this.positions = positions;
    }


    public void set(int num){
        servo.setPosition(positions[num]);
    }
    public double get(int num){
        return positions[num];
    }
}
