package org.firstinspires.ftc.teamcode.libraries.robotPeripherals;
import com.qualcomm.robotcore.hardware.*;


public class Elevator {
    private DcMotor elevator1;
    private DcMotor elevator2;
    private final int LOW_POSITION = 0;
    private final int HIGH_POSITION = 3300;
    private final int LOAD_POSITION = 100;
    private final int HANG_POSITION = 1450;

    public void top() {
        elevator2.setTargetPosition(HIGH_POSITION);
        elevator1.setTargetPosition(HIGH_POSITION);
        elevator1.setPower(1);
        elevator2.setPower(1);
        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void bottom() {
        elevator2.setTargetPosition(LOW_POSITION);
        elevator1.setTargetPosition(LOW_POSITION);
        elevator1.setPower(1);
        elevator2.setPower(1);
        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void hang() {
        elevator2.setTargetPosition(HANG_POSITION);
        elevator1.setTargetPosition(HANG_POSITION);
        elevator1.setPower(1);
        elevator2.setPower(1);
        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void load() {
        elevator2.setTargetPosition(LOAD_POSITION);
        elevator1.setTargetPosition(LOAD_POSITION);
        elevator1.setPower(1);
        elevator2.setPower(1);
        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //returns what percentage the slide is up
    //0 being at bottom, 1 being at top
    public double getPosition() {

        double elevator1PercentUp = ((double)(elevator1.getCurrentPosition()-LOW_POSITION)) /
                ((double)(HIGH_POSITION-LOW_POSITION));

        double elevator2PercentUp = ((double)(elevator2.getCurrentPosition()-LOW_POSITION)) /
                ((double)(HIGH_POSITION-LOW_POSITION));
        return (elevator1PercentUp+elevator2PercentUp)/2;
    }

    //pass in a value between 0 and 1 representing
    //what percentage the elevator should be up,
    //0 low, 1 high
    public void setPosition(double percent) {

        if(percent < 0 || percent > 1){
            return;
        }
        int targetPosition = (int)(LOW_POSITION+
                percent*((double)(HIGH_POSITION-LOW_POSITION)));
        elevator1.setTargetPosition(targetPosition);
        elevator2.setTargetPosition(targetPosition);
        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public Elevator(HardwareMap hardwareMap) {
        elevator1 = hardwareMap.get(DcMotor.class, "elavator1");
        elevator1.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator2 = hardwareMap.get(DcMotor.class, "elavator2");
        elevator2.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
