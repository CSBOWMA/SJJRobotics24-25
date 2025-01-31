package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.libraries.vector.Vector2D;
import org.firstinspires.ftc.teamcode.libraries.MovementCurves.MovementCurves;


@TeleOp
public class RobotFullAbsolute extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        IMU  imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);
        imu.resetYaw();
        //change SLOWSPEED to change how dpad works
        final double SLOWSPEED = .2;

        //change value to change the speed of joysticks
        final double TOTALSPEED = 1;

        double speed;
        double strafe;
        double turn = 0;


        double currentFacing;
        double difference;

        Vector2D direction = new Vector2D(0, 0);
        Vector2D toGo = new Vector2D(0,0);

        DcMotor backRightDrive = null;
        DcMotor frontRightDrive = null;
        DcMotor frontLeftDrive = null;
        DcMotor backLeftDrive = null;
        DcMotor odom = null;

        frontRightDrive = hardwareMap.get(DcMotor.class, "frontright");
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);

        backRightDrive = hardwareMap.get(DcMotor.class, "backright");
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontleft");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        backLeftDrive = hardwareMap.get(DcMotor.class, "backleft");
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);


        odom = hardwareMap.get(DcMotor.class, "straight");
        odom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        Servo outtakeAngle;
        final double OUTTAKE_ANGLE_DROP_POSITION = .589+.03;
        final double OUTTAKE_ANGLE_LOAD_POSITION = .441+0.028;
        final double OUTTAKE_ANGLE_READY_LOAD_POSITION = .443+0.028;

        Servo outtakeClaw;
        final double OUTTAKE_CLAW_OPEN_POSITION = 0.2;
        final double OUTTAKE_CLAW_CLOSED_POSITION = 0.34;


        Servo intakeAngle;
        final double INTAKE_ANGLE_SEARCH_POSITION;
        final double INTAKE_ANGLE_LOAD_POSITION = .75;
        final double INTAKE_ANGLE_GRAB_POSITION = .06;

        Servo intakeClaw;
        final double INTAKE_CLAW_OPEN_POSITION = .0;
        final double INTAKE_CLAW_CLOSED_POSITION = 0.052;

        Servo intakePivot;
        final double INTAKE_PIVOT_HIGH_TURN_POSITION;
        final double INTAKE_PIVOT_LOW_TURN_POSITION;
        final double INTAKE_PIVOT_PASS_POSITION;
        boolean positiveRotate = true;

        //private Servo intakeSlide1; needs to be implemented
        //private Servo intakeSlide2; needs to be implemented

        outtakeAngle = hardwareMap.get(Servo.class, "outtakeAngle");
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        outtakeAngle.setPosition(OUTTAKE_ANGLE_LOAD_POSITION);
        outtakeClaw.setPosition(OUTTAKE_CLAW_CLOSED_POSITION);
        intakeAngle = hardwareMap.get(Servo.class, "intakeAngle");
        intakeAngle.setPosition(INTAKE_ANGLE_LOAD_POSITION);
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeClaw.setPosition(INTAKE_CLAW_OPEN_POSITION);


        Servo slide1;
        Servo slide2;

        double slide1Speed;
        double slide2Speed;

        final double SLIDE_ONE_FAR_POSITION;
        final double SLIDE_ONE_CLOSE_POSITION;

        final double SLIDE_TWO_FAR_POSITION;
        final double SLIDE_TWO_CLOSE_POSITION;
        DcMotor elevator1;
        DcMotor elevator2;



        elevator1 = hardwareMap.get(DcMotor.class, "elavator1");
        elevator1.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2 = hardwareMap.get(DcMotor.class, "elavator2");
        elevator2.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        final int LOW_ELEVATOR_POSITION = 0;
        final int HIGH_ELEVATOR_POSITION = 3300;

        RevColorSensorV3 frontSensor;
        RevColorSensorV3 backSensor;
        final int COLORTHRESHOLD = 128;
        boolean XPressed = false;

        int currentMode = 0;
        final int DEFAULTMODE = 0;
        final int SEARCHMODE = 1;
        final int GRABMODE = 2;
        final int PASSMODE = 3;
        final int ELEVATORMODE = 4;
        final int READYDROPMODE = 5;
        final int DROPMODE = 6;




        //   frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        //   frontRightDrive.setDirection(DcMotor.Direction.REVERSE);

        //   backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        //   backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        //   frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        //   frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        //   backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        //   backLeftDrive.setDirection(DcMotor.Direction.FORWARD);



        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //all drive control, joysticks
            currentFacing = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            toGo.setVector(-gamepad1.left_stick_y, -gamepad1.left_stick_x);

            toGo.setRelative(currentFacing);

            speed = toGo.getI() * TOTALSPEED;
            speed = gamepad1.dpad_up ? speed+SLOWSPEED : speed;
            speed = gamepad1.dpad_down ? speed-SLOWSPEED : speed;

            strafe = toGo.getJ() * TOTALSPEED;
            strafe = gamepad1.dpad_right ? strafe+SLOWSPEED : strafe;
            strafe = gamepad1.dpad_left ? strafe-SLOWSPEED : strafe;

            direction.setVector(-gamepad1.right_stick_y, gamepad1.right_stick_x);

            difference = Math.toDegrees(currentFacing-direction.getRadians());


            slide1Speed = gamepad2.left_stick_y*.1;
            slide2Speed = gamepad2.left_stick_y*.1;

            if (difference > 180) {
                difference -= 360;
            }

            if (difference < -180) {
                difference += 360;
            }

            if (difference > 5 && difference < 180) {
                turn = MovementCurves.circleCurve(difference/360);
            } else if (difference < -5 && difference > -180) {

                turn = -MovementCurves.circleCurve(-difference/360);

            } else {
                turn = 0;
            }

            turn *= direction.getMagnitude();


            telemetry.addData("ticks", odom.getCurrentPosition());

            if (gamepad1.a) {
                currentMode = SEARCHMODE;
            }
            switch (currentMode) {
                case SEARCHMODE:

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    if (intakePivot.getPosition() > INTAKE_PIVOT_HIGH_TURN_POSITION - .02) {
                        positiveRotate = false;
                    }

                    if (intakePivot.getPosition() < INTAKE_PIVOT_LOW_TURN_POSITION + .02) {
                        positiveRotate = true;
                    }

                    if (frontSensor.red() > COLORTHRESHOLD && backSensor.red() > COLORTHRESHOLD
                            || frontSensor.green() > COLORTHRESHOLD && backSensor.green() > COLORTHRESHOLD) {
                        intakePivot.setPosition(intakePivot.getPosition());
                        intakeAngle.setPosition(INTAKE_ANGLE_GRAB_POSITION);
                        if (!gamepad1.a) {
                            currentMode = GRABMODE;
                        }
                    }

                    else if (positiveRotate) {
                        intakePivot.setPosition(INTAKE_PIVOT_HIGH_TURN_POSITION);

                    } else {
                        intakePivot.setPosition(INTAKE_PIVOT_LOW_TURN_POSITION);
                    }

                    elevator1.setTargetPosition(LOW_ELEVATOR_POSITION);
                    elevator2.setTargetPosition(LOW_ELEVATOR_POSITION);
                    elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    outtakeAngle.setPosition(OUTTAKE_ANGLE_LOAD_POSITION);
                    outtakeClaw.setPosition(OUTTAKE_CLAW_OPEN_POSITION);
                    intakeClaw.setPosition(INTAKE_CLAW_OPEN_POSITION);
                    intakeAngle.setPosition(INTAKE_ANGLE_SEARCH_POSITION);

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    break;
                case GRABMODE:
                    speed = 0;
                    strafe = 0;
                    turn = 0;
                    slide1Speed = 0;
                    slide2Speed = 0;

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    intakeAngle.setPosition(INTAKE_ANGLE_GRAB_POSITION);

                    if(intakeAngle.getPosition() > INTAKE_ANGLE_GRAB_POSITION - .02
                            && intakeAngle.getPosition() < INTAKE_ANGLE_GRAB_POSITION + .02) {
                        intakeClaw.setPosition(INTAKE_CLAW_CLOSED_POSITION);
                    }

                    if (intakeClaw.getPosition() > INTAKE_CLAW_CLOSED_POSITION -.02
                    && intakeClaw.getPosition() < INTAKE_CLAW_CLOSED_POSITION + .02 && frontSensor.red() > COLORTHRESHOLD && backSensor.red() > COLORTHRESHOLD
                            || frontSensor.green() > COLORTHRESHOLD && backSensor.green() > COLORTHRESHOLD) {
                        currentMode = PASSMODE;
                    } else if (intakeClaw.getPosition() > INTAKE_CLAW_CLOSED_POSITION -.02
                    && intakeClaw.getPosition() < INTAKE_CLAW_CLOSED_POSITION + .02) {
                        currentMode = SEARCHMODE;
                    }

                    elevator1.setTargetPosition(LOW_ELEVATOR_POSITION);
                    elevator2.setTargetPosition(LOW_ELEVATOR_POSITION);
                    elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    outtakeAngle.setPosition(OUTTAKE_ANGLE_LOAD_POSITION);
                    outtakeClaw.setPosition(OUTTAKE_CLAW_OPEN_POSITION);
                    break;
                case PASSMODE:

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    intakeAngle.setPosition(INTAKE_ANGLE_LOAD_POSITION);
                   if(!(intakeAngle.getPosition() < INTAKE_ANGLE_LOAD_POSITION + .02
                    && intakeAngle.getPosition() > INTAKE_ANGLE_LOAD_POSITION - .02)) {
                       outtakeAngle.setPosition(OUTTAKE_ANGLE_LOAD_POSITION);
                       outtakeClaw.setPosition(OUTTAKE_CLAW_OPEN_POSITION);
                       elevator1.setTargetPosition(LOW_ELEVATOR_POSITION);
                       elevator2.setTargetPosition(LOW_ELEVATOR_POSITION);
                       elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                       elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                   } else if (outtakeClaw.getPosition() < OUTTAKE_CLAW_CLOSED_POSITION + .02
                   && outtakeClaw.getPosition() > OUTTAKE_CLAW_CLOSED_POSITION - .02){
                       outtakeClaw.setPosition(OUTTAKE_CLAW_OPEN_POSITION);
                   } else if (intakeClaw.getPosition() < INTAKE_CLAW_OPEN_POSITION + .02
                   && intakeClaw.getPosition() > INTAKE_CLAW_OPEN_POSITION - .02){
                       currentMode = ELEVATORMODE;
                   }

                    break;
                case ELEVATORMODE:
                    intakeAngle.setPosition(INTAKE_ANGLE_LOAD_POSITION);
                    elevator1.setTargetPosition(HIGH_ELEVATOR_POSITION);
                    elevator2.setTargetPosition(HIGH_ELEVATOR_POSITION);
                    elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    outtakeAngle.setPosition(OUTTAKE_ANGLE_DROP_POSITION);
                    outtakeClaw.setPosition(OUTTAKE_CLAW_CLOSED_POSITION);

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    if (elevator1.getCurrentPosition() > HIGH_ELEVATOR_POSITION-50
                    && elevator2.getCurrentPosition() > HIGH_ELEVATOR_POSITION-50) {
                        currentMode = READYDROPMODE;
                    }

                    break;
                case READYDROPMODE:

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    if (gamepad1.b) {
                        currentMode = DROPMODE;
                    } else {

                        intakeAngle.setPosition(INTAKE_ANGLE_LOAD_POSITION);
                        elevator1.setTargetPosition(HIGH_ELEVATOR_POSITION);
                        elevator2.setTargetPosition(HIGH_ELEVATOR_POSITION);
                        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        outtakeAngle.setPosition(OUTTAKE_ANGLE_DROP_POSITION);
                        outtakeClaw.setPosition(OUTTAKE_CLAW_CLOSED_POSITION);
                    }
                    break;

                case DROPMODE:
                    outtakeClaw.setPosition(OUTTAKE_CLAW_OPEN_POSITION);

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    speed = 0;
                    strafe = 0;
                    turn = 0;
                    if (outtakeClaw.getPosition() < OUTTAKE_CLAW_CLOSED_POSITION + .02
                    && outtakeClaw.getPosition() > OUTTAKE_ANGLE_READY_LOAD_POSITION - .02) {
                        currentMode = DEFAULTMODE;
                    }
                default:

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    intakePivot.setPosition(INTAKE_PIVOT_PASS_POSITION);
                    intakeClaw.setPosition(INTAKE_CLAW_OPEN_POSITION);
                    intakeAngle.setPosition(INTAKE_ANGLE_LOAD_POSITION);
                    outtakeAngle.setPosition(OUTTAKE_ANGLE_LOAD_POSITION);
                    outtakeClaw.setPosition(OUTTAKE_CLAW_OPEN_POSITION);
                    elevator1.setTargetPosition(LOW_ELEVATOR_POSITION);
                    elevator2.setTargetPosition(LOW_ELEVATOR_POSITION);
                    elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }


            frontLeftDrive.setPower(speed + strafe + turn);
            frontRightDrive.setPower(speed - strafe - turn);
            backLeftDrive.setPower(speed - strafe + turn);
            backRightDrive.setPower(speed + strafe - turn);
            //end drive

            //the rest of the code goes here

        }
    }
}