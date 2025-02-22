package org.firstinspires.ftc.teamcode.libraries.robotPeripherals;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeClaw {
    Servo intakeClaw;
    private final double INTAKE_CLAW_OPEN_POSITION = .0;
    private final double INTAKE_CLAW_CLOSED_POSITION = 0.158;

    public void open() {
        intakeClaw.setPosition(INTAKE_CLAW_OPEN_POSITION);
    }

    public void close() {
        intakeClaw.setPosition(INTAKE_CLAW_CLOSED_POSITION);
    }

    //returns the percentage the claw is closed
    //0 is open, 1 is closed
    public double getPosition() {
        return (intakeClaw.getPosition() - INTAKE_CLAW_OPEN_POSITION) /
                (INTAKE_CLAW_CLOSED_POSITION - INTAKE_CLAW_OPEN_POSITION);
    }

    //pass in a value between 0 and 1 representing
    //what percentage the claw should be closed
    public void setPosition(double percent) {
        if(percent < 0 || percent > 1){
            return;
        }
        double targetPosition = INTAKE_CLAW_OPEN_POSITION+
                percent*(INTAKE_CLAW_CLOSED_POSITION-INTAKE_CLAW_OPEN_POSITION);
        intakeClaw.setPosition(targetPosition);
    }

    public IntakeClaw(HardwareMap hardwareMap) {
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
    }
}
