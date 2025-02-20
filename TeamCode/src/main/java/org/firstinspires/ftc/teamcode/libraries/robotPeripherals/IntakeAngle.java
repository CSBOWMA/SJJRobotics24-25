package org.firstinspires.ftc.teamcode.libraries.robotPeripherals;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeAngle {
    Servo intakeAngle1;
    Servo intakeAngle2;

    final double INTAKE_ONE_ANGLE_SEARCH_POSITION = 0.0685;
    final double INTAKE_ONE_ANGLE_LOAD_POSITION = .73;
    final double INTAKE_ONE_ANGLE_GRAB_POSITION = .03;

    final double INTAKE_TWO_ANGLE_SEARCH_POSITION = 0.685;
    final double INTAKE_TWO_ANGLE_LOAD_POSITION = .025;
    final double INTAKE_TWO_ANGLE_GRAB_POSITION = .72;

    public void search() {
        intakeAngle1.setPosition(INTAKE_ONE_ANGLE_SEARCH_POSITION);
        intakeAngle2.setPosition(INTAKE_TWO_ANGLE_SEARCH_POSITION);
    }

    public void grab() {
        intakeAngle1.setPosition(INTAKE_ONE_ANGLE_GRAB_POSITION);
        intakeAngle2.setPosition(INTAKE_TWO_ANGLE_GRAB_POSITION);
    }

    public void load() {
        intakeAngle1.setPosition(INTAKE_ONE_ANGLE_LOAD_POSITION);
        intakeAngle2.setPosition(INTAKE_TWO_ANGLE_LOAD_POSITION);
    }
    //returns what percentage the angle is out
    //0 load position, 1 grab position
    public double getPosition() {
       double intakeAngle1PercentOut = (intakeAngle1.getPosition()-INTAKE_ONE_ANGLE_LOAD_POSITION) /
               (INTAKE_ONE_ANGLE_GRAB_POSITION-INTAKE_ONE_ANGLE_LOAD_POSITION);
       double intakeAngle2PercentOut = (intakeAngle2.getPosition()-INTAKE_TWO_ANGLE_LOAD_POSITION) /
               (INTAKE_TWO_ANGLE_GRAB_POSITION-INTAKE_TWO_ANGLE_LOAD_POSITION);

       return (intakeAngle1PercentOut+intakeAngle2PercentOut)/2;

    }

    //pass in a value between 0 and 1 representing
    //what percentage the claw should be out,
    //0 load position, 1 grab position
    public void setPosition(double percent) {
        if (percent < 0 || percent > 1) {
            return;
        }

        double intakeOneTargetPosition = INTAKE_ONE_ANGLE_LOAD_POSITION +
                percent*(INTAKE_ONE_ANGLE_GRAB_POSITION-INTAKE_ONE_ANGLE_LOAD_POSITION);
        double intakeTwoTargetPosition = INTAKE_TWO_ANGLE_LOAD_POSITION +
                percent*(INTAKE_TWO_ANGLE_GRAB_POSITION-INTAKE_TWO_ANGLE_LOAD_POSITION);

        intakeAngle1.setPosition(intakeOneTargetPosition);
        intakeAngle2.setPosition(intakeTwoTargetPosition);
    }

    public IntakeAngle(HardwareMap hardwareMap) {

        intakeAngle1 = hardwareMap.get(Servo.class, "intakeAngle");
        intakeAngle2 = hardwareMap.get(Servo.class, "intakeAngle2");
    }
}
