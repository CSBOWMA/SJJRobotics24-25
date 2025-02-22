package org.firstinspires.ftc.teamcode.libraries.robotPeripherals;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakePivot {


    Servo intakePivot;
    private final double INTAKE_PIVOT_HIGH_TURN_POSITION = .58;
    private final double INTAKE_PIVOT_LOW_TURN_POSITION = .49;
    private final double INTAKE_PIVOT_PASS_POSITION = 0.49;

    public void pass() {
      intakePivot.setPosition(INTAKE_PIVOT_PASS_POSITION);
    }

    public void highTurn() {
        intakePivot.setPosition(INTAKE_PIVOT_HIGH_TURN_POSITION);
    }

    public void lowTurn() {
        intakePivot.setPosition(INTAKE_PIVOT_LOW_TURN_POSITION);
    }


    //returns a what percentage the claw is turned
    //low turn is 0, high turn is 1
    public double getPosition() {
        return (intakePivot.getPosition()-INTAKE_PIVOT_LOW_TURN_POSITION) /
                (INTAKE_PIVOT_HIGH_TURN_POSITION-INTAKE_PIVOT_LOW_TURN_POSITION);
    }

    //takes in a value between 0 and 1 that represents
    //how turned the pivot is, 0 being pass position, and 1
    //being perpendicular to pass position
    public void setPosition(double percent) {

        if(percent < 0 || percent > 1){
            return;
        }
        double targetPosition = INTAKE_PIVOT_LOW_TURN_POSITION +
                percent * (INTAKE_PIVOT_HIGH_TURN_POSITION - INTAKE_PIVOT_LOW_TURN_POSITION);
        intakePivot.setPosition(targetPosition);
    }

    public IntakePivot(HardwareMap hardwareMap) {
        intakePivot = hardwareMap.get(Servo.class, "intakeRotate");
    }
}
