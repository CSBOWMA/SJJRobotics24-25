package org.firstinspires.ftc.teamcode.libraries.robotPeripherals;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeClaw {

    Servo outtakeClaw;

    private final double OUTTAKE_CLAW_OPEN_POSITION = 0.2;
    private final double OUTTAKE_CLAW_CLOSED_POSITION = 0.34;
    final double OUTTAKE_CLAW_PREDROP_POSITION = .3;

    public void open() {
        outtakeClaw.setPosition(OUTTAKE_CLAW_OPEN_POSITION);
    }

    public void close() {
        outtakeClaw.setPosition(OUTTAKE_CLAW_CLOSED_POSITION);
    }

    public void predrop() {
        outtakeClaw.setPosition(OUTTAKE_CLAW_PREDROP_POSITION);
    }

    //returns a value between 0 and 1 representing how closed the claw is
    //0 being open 1 being closed
    public double getPosition() {
        return (outtakeClaw.getPosition()-OUTTAKE_CLAW_OPEN_POSITION) /
                (OUTTAKE_CLAW_CLOSED_POSITION-OUTTAKE_CLAW_OPEN_POSITION);
    }

    //pass in a value between 0 and 1 representing
    //what percentage the claw should be closed
    public void setPosition(double percent) {
        if(percent < 0 || percent > 1) {
            return;
        }
        double targetPosition = OUTTAKE_CLAW_OPEN_POSITION +
                percent * (OUTTAKE_CLAW_CLOSED_POSITION-OUTTAKE_CLAW_OPEN_POSITION);
        outtakeClaw.setPosition(targetPosition);
    }

   public OuttakeClaw(HardwareMap hardwareMap) {
       outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
   }

}
