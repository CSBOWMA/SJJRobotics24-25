package org.firstinspires.ftc.teamcode.libraries.robotPeripherals;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeAngle {
    Servo outtakeAngle;
    private final double OUTTAKE_ANGLE_DROP_POSITION = .59;
    private final double OUTTAKE_ANGLE_LOAD_POSITION = .445;
    final double OUTTAKE_ANGLE_PREDROP_POSITION = 0.62;
    final double OUTTAKE_ANGLE_POSTDROP_POSITION = 0.7;


    public void predrop() {
        outtakeAngle.setPosition(OUTTAKE_ANGLE_PREDROP_POSITION);
    }

    public void postdrop() {
        outtakeAngle.setPosition(OUTTAKE_ANGLE_POSTDROP_POSITION);
    }

    public void load() {
        outtakeAngle.setPosition(OUTTAKE_ANGLE_LOAD_POSITION);
    }

    public void drop() {
        outtakeAngle.setPosition(OUTTAKE_ANGLE_DROP_POSITION);
    }

    //returns a value between 0 and 1 representing
    //how far the claw is out, 0 load position, 1 drop position
    public double getPosition() {
        return (outtakeAngle.getPosition()-OUTTAKE_ANGLE_LOAD_POSITION) /
                (OUTTAKE_ANGLE_DROP_POSITION-OUTTAKE_ANGLE_LOAD_POSITION);
    }

    //pass in a value between 0 and 1 representing
    //what percentage the angle should be out,
    //0 is load position, 1 is drop position
    public void setPosition(double percent) {

        if(percent < 0 || percent > 1) {
            return;
        }
        double targetPosition = OUTTAKE_ANGLE_LOAD_POSITION +
                percent * (OUTTAKE_ANGLE_DROP_POSITION-OUTTAKE_ANGLE_LOAD_POSITION);
        outtakeAngle.setPosition(targetPosition);
    }

    public OuttakeAngle(HardwareMap hardwareMap) {
        outtakeAngle = hardwareMap.get(Servo.class, "outtakeAngle");
    }
}
