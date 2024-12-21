package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestRobotFuncs extends LinearOpMode {

    // todo: write your code here
    private Servo servo;
    private CRServo crServo;
    private DcMotor motor;

    @Override
    public void runOpMode(){
        servo = hardwareMap.get(Servo.class, "servo");
        crServo = hardwareMap.get(CRServo.class, "crservo");

        motor = hardwareMap.get(DcMotor.class, "motor");
        servo.setPosition(0);

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a){
                servo.setPosition(servo.getPosition()+0.0001);
            }
            if(gamepad1.b){
                servo.setPosition(servo.getPosition()-0.0001);
            } if (gamepad1.x){
                crServo.setPower(1.0);
            } if(gamepad1.y){
                crServo.setPower(-1.0);
            }
            motor.setPower(gamepad1.left_stick_y);
            telemetry.addData("motor Power: ", motor.getPower());
            telemetry.update();
        }
    }


}
