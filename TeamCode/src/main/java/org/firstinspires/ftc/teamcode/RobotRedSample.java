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
public class RobotRedSample extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        IMU  imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);
        imu.resetYaw();
        //change SLOWSPEED to change how dpad works
        final double SLOWSPEED = .2;

        long timer = 0;
        double timerSeconds = 0;
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
        final double OUTTAKE_ANGLE_DROP_POSITION = 0.59;
        final double OUTTAKE_ANGLE_PREDROP_POSITION = 0.64;
        final double OUTTAKE_ANGLE_LOAD_POSITION = .441+.04;

        Servo outtakeClaw;
        final double OUTTAKE_CLAW_OPEN_POSITION = 0.2;
        final double OUTTAKE_CLAW_PREDROP_POSITION = .3;
        final double OUTTAKE_CLAW_CLOSED_POSITION = 0.34;


        Servo intakeAngle1;
        intakeAngle1 = hardwareMap.get(Servo.class, "intakeAngle");
        Servo intakeAngle2;
        intakeAngle2 = hardwareMap.get(Servo.class, "intakeAngle2");

        final double INTAKE_ONE_ANGLE_SEARCH_POSITION = 0.07;
        final double INTAKE_ONE_ANGLE_LOAD_POSITION = .73;
        final double INTAKE_ONE_ANGLE_GRAB_POSITION = .04;

        final double INTAKE_TWO_ANGLE_SEARCH_POSITION = 0.68;
        final double INTAKE_TWO_ANGLE_LOAD_POSITION = .02;
        final double INTAKE_TWO_ANGLE_GRAB_POSITION = .71;
        Servo intakeClaw;
        final double INTAKE_CLAW_OPEN_POSITION = .0;
        final double INTAKE_CLAW_CLOSED_POSITION = .158;

        Servo intakePivot;
        final double INTAKE_PIVOT_HIGH_TURN_POSITION = .553;
        final double INTAKE_PIVOT_LOW_TURN_POSITION = .49;
        final double INTAKE_PIVOT_POSITION_DIFFERENCE = INTAKE_PIVOT_HIGH_TURN_POSITION-INTAKE_PIVOT_LOW_TURN_POSITION;
        final double INTAKE_PIVOT_PASS_POSITION = 0.49;
        boolean positiveRotate = true;
        intakePivot = hardwareMap.get(Servo.class, "intakeRotate");
        //private Servo intakeSlide1; needs to be implemented
        //private Servo intakeSlide2; needs to be implemented

        outtakeAngle = hardwareMap.get(Servo.class, "outtakeAngle");
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
//        timer = System.nanoTime();

//        while(timer + 2_000_000_000 > System.nanoTime());

        intakeAngle1 = hardwareMap.get(Servo.class, "intakeAngle");
        intakeAngle2 = hardwareMap.get(Servo.class, "intakeAngle2");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");

        Servo slide1;
        Servo slide2;

        slide1 = hardwareMap.get(Servo.class, "intakeSlide1");
        slide2 = hardwareMap.get(Servo.class, "intakeSlide2");
        double slide1Speed;
        double slide2Speed;

        final double SLIDE_ONE_FAR_POSITION = .35;
        final double SLIDE_ONE_CLOSE_POSITION = 0;
        final double SLIDE_ONE_PREPASS_POSITION = .12;
        final double SLIDE_ONE_PASS_POSITION = .04;

        final double SLIDE_TWO_FAR_POSITION = .65;
        final double SLIDE_TWO_CLOSE_POSITION = 1;
        final double SLIDE_TWO_PREPASS_POSITION = .88;
        final double SLIDE_TWO_PASS_POSITION = .96;

        double slide1Pos = SLIDE_ONE_PASS_POSITION;
        double slide2Pos = SLIDE_TWO_PASS_POSITION;
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

        frontSensor = hardwareMap.get(RevColorSensorV3.class, "c1");
        backSensor = hardwareMap.get(RevColorSensorV3.class, "c2");

        final int COLORTHRESHOLD = 120;
        boolean XPressed = false;

        int currentMode = 0;
        final int DEFAULTMODE = 0;
        final int SEARCHMODE = 1;
        final int GRABMODE = 2;
        final int PASSMODE = 3;
        final int ELEVATORMODE = 4;
        final int READYDROPMODE = 5;
        final int DROPMODE = 6;

        long currentTime;
        double currentTimeSeconds;

        //   frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        //   frontRightDrive.setDirection(DcMotor.Direction.REVERSE);

        //   backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        //   backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        //   frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        //   frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        //   backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        //   backLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        outtakeAngle.setPosition(OUTTAKE_ANGLE_DROP_POSITION);
        outtakeClaw.setPosition(OUTTAKE_CLAW_CLOSED_POSITION);
        timer = System.nanoTime();
        slide1.setPosition(SLIDE_ONE_PASS_POSITION);
        slide2.setPosition(SLIDE_TWO_PASS_POSITION);
        while(timer + 1_000_000_000 > System.nanoTime());
        intakeAngle1.setPosition(INTAKE_ONE_ANGLE_LOAD_POSITION);
        intakeAngle2.setPosition(INTAKE_TWO_ANGLE_LOAD_POSITION);
        intakeClaw.setPosition(INTAKE_CLAW_CLOSED_POSITION);
        outtakeAngle.setPosition(OUTTAKE_ANGLE_LOAD_POSITION);
        intakePivot.setPosition(INTAKE_PIVOT_PASS_POSITION);



        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //all drive control, joysticks
            currentTime = System.nanoTime();
            currentTimeSeconds = currentTime/1_000_000_000.0;
            currentFacing = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            speed = -gamepad1.left_stick_y*0.8;
            strafe = gamepad1.left_stick_x*0.8;
            turn = gamepad1.right_stick_x*0.8;

            if (gamepad2.triangle && currentMode != READYDROPMODE) {
                currentMode = DEFAULTMODE;
            }

            if (gamepad2.a && currentMode != READYDROPMODE) {
                currentMode = SEARCHMODE;
                timer = currentTime;
                timerSeconds = currentTimeSeconds;
            }

            telemetry.addData("currentMode", currentMode);
            telemetry.addData("sens1R", frontSensor.red());
            telemetry.addData("sens2R", backSensor.red());
            telemetry.addData("currentSeconds", currentTimeSeconds);
            telemetry.addData("seconds", timerSeconds);

            telemetry.update();

            switch (currentMode) {
                case SEARCHMODE:

                    speed *= .3;
                    strafe *= .3;
                    turn *= .3;

                    if (gamepad2.right_trigger > 0.2) {
                        slide1.setPosition(slide1.getPosition() - 0.0075);//.decrease();
                        slide2.setPosition(slide2.getPosition() + 0.0075);//.increase();
                    }
                    if (gamepad2.left_trigger > 0.2) {
                        slide1.setPosition(slide1.getPosition() + 0.0075);//.increase();
                        slide2.setPosition(slide2.getPosition() - 0.0075);//.decrease();
                    }

//                    slide1Speed *= .3;
//                    slide2Speed *= .3;
//                    slide1Pos += slide1Speed;
//                    slide2Pos += slide2Speed;
//                    slide1.setPosition(slide1Pos);
//                    slide2.setPosition(slide2Pos);
                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


                    if ((frontSensor.red() > COLORTHRESHOLD && backSensor.red() > COLORTHRESHOLD
                            || frontSensor.green() > COLORTHRESHOLD && backSensor.green() > COLORTHRESHOLD)
                            && timerSeconds + 1 < currentTimeSeconds || gamepad2.square) {
                        intakePivot.setPosition(intakePivot.getPosition());
                        intakeAngle1.setPosition(INTAKE_ONE_ANGLE_GRAB_POSITION);

                        intakeAngle2.setPosition(INTAKE_TWO_ANGLE_GRAB_POSITION);
                        currentMode = GRABMODE;
                        timer = currentTime;
                        timerSeconds = currentTimeSeconds;
                    }


                    intakePivot.setPosition(INTAKE_PIVOT_LOW_TURN_POSITION - .02+
                            (MovementCurves.linear(((double)(currentTime%1_500_000_000))/1_500_000_000.0)*(INTAKE_PIVOT_POSITION_DIFFERENCE+.02)));
                    telemetry.addData("SWITCH", (currentTimeSeconds*2)%5);

                    elevator1.setTargetPosition(LOW_ELEVATOR_POSITION);
                    elevator2.setTargetPosition(LOW_ELEVATOR_POSITION);
                    elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    outtakeAngle.setPosition(OUTTAKE_ANGLE_LOAD_POSITION);
                    outtakeClaw.setPosition(OUTTAKE_CLAW_OPEN_POSITION);
                    intakeClaw.setPosition(INTAKE_CLAW_OPEN_POSITION);
                    intakeAngle1.setPosition(INTAKE_ONE_ANGLE_SEARCH_POSITION);
                    intakeAngle2.setPosition(INTAKE_TWO_ANGLE_SEARCH_POSITION);

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    break;
                case GRABMODE:
                    speed = 0;
                    strafe = 0;
                    turn = 0;
//                    slide1Speed = 0;
//                    slide2Speed = 0;
//
                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    intakeAngle1.setPosition(INTAKE_ONE_ANGLE_GRAB_POSITION);
                    intakeAngle2.setPosition(INTAKE_TWO_ANGLE_GRAB_POSITION);


                    if(timerSeconds + .25 < currentTimeSeconds) {
                        intakeClaw.setPosition(INTAKE_CLAW_CLOSED_POSITION);
                    }

                    if (timerSeconds + .5 < currentTimeSeconds && (frontSensor.red() > COLORTHRESHOLD && backSensor.red() > COLORTHRESHOLD
                            || frontSensor.green() > COLORTHRESHOLD && backSensor.green() > COLORTHRESHOLD)) {
                        currentMode = PASSMODE;
                        timer = currentTime;
                        timerSeconds = currentTimeSeconds;
                    } else if (timerSeconds + .5 < currentTimeSeconds) {
                        currentMode = SEARCHMODE;
                    }

                    elevator1.setTargetPosition(LOW_ELEVATOR_POSITION);
                    elevator2.setTargetPosition(LOW_ELEVATOR_POSITION);
                    elevator1.setPower(1);
                    elevator2.setPower(1);
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
                    intakeAngle1.setPosition(INTAKE_ONE_ANGLE_LOAD_POSITION);
                    intakeAngle2.setPosition(INTAKE_TWO_ANGLE_LOAD_POSITION);
                    intakePivot.setPosition(INTAKE_PIVOT_PASS_POSITION);


                    if (timerSeconds + 4 < currentTimeSeconds) {
                        currentMode = ELEVATORMODE;
                        timer = currentTime;
                        timerSeconds = currentTimeSeconds;
                    }

                    else if (timerSeconds + 3 < currentTimeSeconds) {
                        intakeClaw.setPosition(INTAKE_CLAW_OPEN_POSITION);

                    } else if (timerSeconds + 2 < currentTimeSeconds) {

                        slide1.setPosition(SLIDE_ONE_PASS_POSITION);
                        slide2.setPosition(SLIDE_TWO_PASS_POSITION);
                        outtakeClaw.setPosition(OUTTAKE_CLAW_CLOSED_POSITION);
                    } else if (timerSeconds + 1.5 < currentTimeSeconds) {

                        slide1.setPosition(SLIDE_ONE_PASS_POSITION);
                        slide2.setPosition(SLIDE_TWO_PASS_POSITION);

                    } else if(timerSeconds + 1 < currentTimeSeconds) {
                        slide1.setPosition(SLIDE_ONE_PREPASS_POSITION);
                        slide2.setPosition(SLIDE_TWO_PREPASS_POSITION);
                        outtakeAngle.setPosition(OUTTAKE_ANGLE_LOAD_POSITION);
                        outtakeClaw.setPosition(OUTTAKE_CLAW_OPEN_POSITION);
                        elevator1.setTargetPosition(LOW_ELEVATOR_POSITION);
                        elevator2.setTargetPosition(LOW_ELEVATOR_POSITION);
                        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    } else {
                        slide1.setPosition(SLIDE_ONE_PREPASS_POSITION);
                        slide2.setPosition(SLIDE_TWO_PREPASS_POSITION);
                    }
                    break;
                case ELEVATORMODE:
                    intakeAngle1.setPosition(INTAKE_ONE_ANGLE_LOAD_POSITION);
                    intakeAngle2.setPosition(INTAKE_TWO_ANGLE_LOAD_POSITION);
                    elevator1.setTargetPosition(HIGH_ELEVATOR_POSITION);
                    elevator2.setTargetPosition(HIGH_ELEVATOR_POSITION);
                    elevator1.setPower(1);
                    elevator2.setPower(1);
                    elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    outtakeAngle.setPosition(OUTTAKE_ANGLE_PREDROP_POSITION);
                    outtakeClaw.setPosition(OUTTAKE_CLAW_CLOSED_POSITION);

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    if (timerSeconds + 1 < currentTimeSeconds) {
                        outtakeClaw.setPosition(OUTTAKE_CLAW_PREDROP_POSITION);
                    } else {
                        outtakeClaw.setPosition(OUTTAKE_CLAW_CLOSED_POSITION);
                    }

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
                    if (gamepad2.circle) {
                        currentMode = DROPMODE;
                        timer = currentTime;
                        timerSeconds = currentTimeSeconds;
                    } else {

                        intakeAngle1.setPosition(INTAKE_ONE_ANGLE_LOAD_POSITION);
                        intakeAngle2.setPosition(INTAKE_TWO_ANGLE_LOAD_POSITION);

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
                    if (timerSeconds + .75 < currentTimeSeconds) {
                        currentMode = DEFAULTMODE;
                    }
                    break;
                default:

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    intakePivot.setPosition(INTAKE_PIVOT_PASS_POSITION);
                    intakeClaw.setPosition(INTAKE_CLAW_OPEN_POSITION);
                    intakeAngle1.setPosition(INTAKE_ONE_ANGLE_LOAD_POSITION);
                    intakeAngle2.setPosition(INTAKE_TWO_ANGLE_LOAD_POSITION);
                    outtakeAngle.setPosition(OUTTAKE_ANGLE_LOAD_POSITION);
                    outtakeClaw.setPosition(OUTTAKE_CLAW_OPEN_POSITION);
                    elevator1.setTargetPosition(LOW_ELEVATOR_POSITION);
                    elevator2.setTargetPosition(LOW_ELEVATOR_POSITION);
                    elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide1.setPosition(SLIDE_ONE_PREPASS_POSITION);
                    slide2.setPosition(SLIDE_TWO_PREPASS_POSITION);
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