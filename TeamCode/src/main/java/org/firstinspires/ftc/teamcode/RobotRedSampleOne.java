package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.libraries.MovementCurves.MovementCurves;



//Designed to handle controls for one person,
//allowing for the other to spot or perform other actions
@TeleOp
public class RobotRedSampleOne extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        IMU  imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);
        imu.resetYaw();
        //change SLOWSPEED to change how dpad works
        //initialize all robot positions, and relevant variables
        final double SLOWSPEED = .2;

        //handle timed events
        long timer = 0;
        double timerSeconds = 0;

        //change value to change the speed of joysticks
        final double TOTALSPEED = 1;

        double speed;
        double strafe;
        double turn = 0;



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
        final double OUTTAKE_ANGLE_PREDROP_POSITION = 0.62;
        final double OUTTAKE_ANGLE_LOAD_POSITION = .441+.04;

        Servo outtakeClaw;
        final double OUTTAKE_CLAW_OPEN_POSITION = 0.2;
        final double OUTTAKE_CLAW_PREDROP_POSITION = .3;
        final double OUTTAKE_CLAW_CLOSED_POSITION = 0.34;


        Servo intakeAngle1;
        intakeAngle1 = hardwareMap.get(Servo.class, "intakeAngle");
        Servo intakeAngle2;
        intakeAngle2 = hardwareMap.get(Servo.class, "intakeAngle2");

        final double INTAKE_ONE_ANGLE_SEARCH_POSITION = 0.065;
        final double INTAKE_ONE_ANGLE_LOAD_POSITION = .725;
        final double INTAKE_ONE_ANGLE_GRAB_POSITION = .03;

        final double INTAKE_TWO_ANGLE_SEARCH_POSITION = 0.685;
        final double INTAKE_TWO_ANGLE_LOAD_POSITION = .025;
        final double INTAKE_TWO_ANGLE_GRAB_POSITION = .72;
        Servo intakeClaw;
        final double INTAKE_CLAW_OPEN_POSITION = .0;
        final double INTAKE_CLAW_CLOSED_POSITION = .158;

        Servo intakePivot;
        final double INTAKE_PIVOT_HIGH_TURN_POSITION = .58;
        final double INTAKE_PIVOT_LOW_TURN_POSITION = .49;
        final double INTAKE_PIVOT_POSITION_DIFFERENCE = INTAKE_PIVOT_HIGH_TURN_POSITION-INTAKE_PIVOT_LOW_TURN_POSITION;
        final double INTAKE_PIVOT_PASS_POSITION = 0.49;
        boolean positiveRotate = true;
        intakePivot = hardwareMap.get(Servo.class, "intakeRotate");

        outtakeAngle = hardwareMap.get(Servo.class, "outtakeAngle");
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");

        intakeAngle1 = hardwareMap.get(Servo.class, "intakeAngle");
        intakeAngle2 = hardwareMap.get(Servo.class, "intakeAngle2");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");

        Servo slide1;
        Servo slide2;

        slide1 = hardwareMap.get(Servo.class, "intakeSlide1");
        slide2 = hardwareMap.get(Servo.class, "intakeSlide2");

        final double SLIDE_ONE_FAR_POSITION = .35;
        final double SLIDE_ONE_CLOSE_POSITION = 0;
        final double SLIDE_ONE_PREPASS_POSITION = .17;
        final double SLIDE_ONE_PASS_POSITION = .035;

        final double SLIDE_TWO_FAR_POSITION = .65;
        final double SLIDE_TWO_CLOSE_POSITION = 1;
        final double SLIDE_TWO_PREPASS_POSITION = .83;
        final double SLIDE_TWO_PASS_POSITION = .965;

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

        final int FRONTDISTANCETHRESHOLD = 40;
        final int BACKDISTANCETHRESHOLD = 35;
        boolean XPressed = false;

        int currentMode = 0;
        final int DEFAULTMODE = 0;
        final int PUSHMODE = 1;
        final int SEARCHMODE = 2;
        final int GRABMODE = 3;
        final int PASSMODE = 4;
        final int ELEVATORMODE = 5;
        final int READYDROPMODE = 6;
        final int DROPMODE = 7;

        long currentTime;
        double currentTimeSeconds;

        outtakeAngle.setPosition(OUTTAKE_ANGLE_DROP_POSITION);
        outtakeClaw.setPosition(OUTTAKE_CLAW_CLOSED_POSITION);
        timer = System.nanoTime();
        slide1.setPosition(SLIDE_ONE_PASS_POSITION);
        slide2.setPosition(SLIDE_TWO_PASS_POSITION);
        while(timer + 1_000_000_000 > System.nanoTime());
        intakeAngle1.setPosition(INTAKE_ONE_ANGLE_LOAD_POSITION);
        intakeAngle2.setPosition(INTAKE_TWO_ANGLE_LOAD_POSITION);
        intakeClaw.setPosition(INTAKE_CLAW_OPEN_POSITION);
        outtakeAngle.setPosition(OUTTAKE_ANGLE_LOAD_POSITION);
        intakePivot.setPosition(INTAKE_PIVOT_PASS_POSITION);

        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //all drive control, joysticks
            currentTime = System.nanoTime();
            currentTimeSeconds = currentTime/1_000_000_000.0;

            speed = -gamepad1.left_stick_y*0.5;
            strafe = gamepad1.left_stick_x*0.5;
            turn = gamepad1.right_stick_x*0.5;

            if (gamepad2.triangle && currentMode != READYDROPMODE) {
                currentMode = DEFAULTMODE;
            }

            if (gamepad2.square&& currentMode != READYDROPMODE) {
                currentMode = GRABMODE;
                timer = 2*currentTime;
                timerSeconds = 2*currentTimeSeconds;
            }

            if (gamepad2.a && currentMode != READYDROPMODE) {
                currentMode = SEARCHMODE;
                timer = currentTime;
                timerSeconds = currentTimeSeconds;
            }

            telemetry.addData("currentMode", currentMode);
            telemetry.addData("sens1R", frontSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("sens2R", backSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("currentSeconds", currentTimeSeconds);
            telemetry.addData("seconds", timerSeconds);

            telemetry.update();

            switch (currentMode) {


                // designed to handle moving blocks around, but grab mode was designed to be manual,
                //so it is instead utilized, left in case automatic functionality is reimplemented
                //  case PUSHMODE:


                //      speed *= .3;
                //      strafe *= .3;
                //      turn *= .5;

                //      if (gamepad2.right_trigger > 0.2) {
                //          slide1.setPosition(slide1.getPosition() + 0.0025);//.decrease();
                //          slide2.setPosition(slide2.getPosition() - 0.0025);//.increase();
                //      }
                //      if (gamepad2.left_trigger > 0.2) {
                //          slide1.setPosition(slide1.getPosition() - 0.0025);//.increase();
                //          slide2.setPosition(slide2.getPosition() + 0.0025);//.decrease();
                //      }
                //      frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                //      backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                //      frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                //      frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                //      intakeAngle1.setPosition(INTAKE_ONE_ANGLE_GRAB_POSITION);
                //      intakeAngle2.setPosition(INTAKE_TWO_ANGLE_GRAB_POSITION);

                //      if (!gamepad2.square) {
                //          currentMode = SEARCHMODE;
                //      }

                //      elevator1.setTargetPosition(LOW_ELEVATOR_POSITION);
                //      elevator2.setTargetPosition(LOW_ELEVATOR_POSITION);
                //      elevator1.setPower(1);
                //      elevator2.setPower(1);
                //      elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //      elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //      outtakeAngle.setPosition(OUTTAKE_ANGLE_LOAD_POSITION);
                //      outtakeClaw.setPosition(OUTTAKE_CLAW_OPEN_POSITION);

                //      break;

                //first state, front claw goes into search position, and oscillates until it finds a block
                //then the robot lowers the claw and goes into grab mode
                //robot moves slowly in this mode to help with accurately finding blocks
                case SEARCHMODE:

                    speed *= .3;
                    strafe *= .4;
                    turn *= .5;

                    if (gamepad2.right_trigger > 0.2) {
                        slide1.setPosition(slide1.getPosition() + 0.0055);//.decrease();
                        slide2.setPosition(slide2.getPosition() - 0.0055);//.increase();

                    }
                    if (gamepad2.left_trigger > 0.2) {
                        slide1.setPosition(slide1.getPosition() - 0.0055);//.increase();
                        slide2.setPosition(slide2.getPosition() + 0.0055);//.decrease();
                    }
                    if (slide1.getPosition() > SLIDE_ONE_FAR_POSITION) {
                        slide1.setPosition(SLIDE_ONE_FAR_POSITION);
                    }
                    if (slide2.getPosition() < SLIDE_TWO_FAR_POSITION) {
                        slide2.setPosition(SLIDE_TWO_FAR_POSITION);
                    }

                    if (slide1.getPosition() < SLIDE_ONE_CLOSE_POSITION) {
                        slide1.setPosition(SLIDE_ONE_CLOSE_POSITION);
                    }
                    if (slide2.getPosition() > SLIDE_TWO_CLOSE_POSITION) {
                        slide2.setPosition(SLIDE_TWO_CLOSE_POSITION);
                    }



                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


                    if (frontSensor.getDistance(DistanceUnit.MM) < FRONTDISTANCETHRESHOLD
                            && backSensor.getDistance(DistanceUnit.MM) < BACKDISTANCETHRESHOLD) {
                        intakePivot.setPosition(intakePivot.getPosition());
                        intakeAngle1.setPosition(INTAKE_ONE_ANGLE_GRAB_POSITION);

                        intakeAngle2.setPosition(INTAKE_TWO_ANGLE_GRAB_POSITION);
                        currentMode = GRABMODE;
                        timer = currentTime*2;
                        timerSeconds = currentTimeSeconds*2;
                    }


                    intakePivot.setPosition(INTAKE_PIVOT_LOW_TURN_POSITION +
                            (MovementCurves.linear(((double)(currentTime%2_500_000_000L))/2_500_000_000.0D)*(INTAKE_PIVOT_POSITION_DIFFERENCE)));
                    telemetry.addData("SWITCH", (currentTimeSeconds*2)%5);


                    //all states the robot should be in when in this mode
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

                //initialized after finding a block or override button is pressed
                //in this mode the claw lowers and stops moving, allowing user to grab block
                case GRABMODE:

                    speed *= .3;
                    strafe *= .4;
                    turn *= .5;

                    if (gamepad2.right_trigger > 0.2) {
                        slide1.setPosition(slide1.getPosition() + 0.0025);//.decrease();
                        slide2.setPosition(slide2.getPosition() - 0.0025);//.increase();
                    }
                    if (gamepad2.left_trigger > 0.2) {
                        slide1.setPosition(slide1.getPosition() - 0.0025);//.increase();
                        slide2.setPosition(slide2.getPosition() + 0.0025);//.decrease();
                    }
                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    intakeAngle1.setPosition(INTAKE_ONE_ANGLE_GRAB_POSITION);
                    intakeAngle2.setPosition(INTAKE_TWO_ANGLE_GRAB_POSITION);


                    //grabs block and gets ready to load
                    if(gamepad2.right_bumper) {
                        intakeClaw.setPosition(INTAKE_CLAW_CLOSED_POSITION);
                        timer = currentTime;
                        timerSeconds = currentTimeSeconds;
                    }

                    if (timerSeconds + .5 < currentTimeSeconds) {
                        currentMode = PASSMODE;
                        timer = currentTime;
                        timerSeconds = currentTimeSeconds;
                    } //else if (timerSeconds + .5 < currentTimeSeconds) {
                    //  currentMode = SEARCHMODE;
                    //}

                    elevator1.setTargetPosition(LOW_ELEVATOR_POSITION);
                    elevator2.setTargetPosition(LOW_ELEVATOR_POSITION);
                    elevator1.setPower(1);
                    elevator2.setPower(1);
                    elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    outtakeAngle.setPosition(OUTTAKE_ANGLE_LOAD_POSITION);
                    outtakeClaw.setPosition(OUTTAKE_CLAW_OPEN_POSITION);
                    break;

                // Block is now grabbed and ready to be passed through the robot,
                //consistent issue is servo unalignment, will need to be fixed in future
                case PASSMODE:

                    //initial positions, lets driver move full speed so they may get ready to place
                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    intakeAngle1.setPosition(INTAKE_ONE_ANGLE_LOAD_POSITION);
                    intakeAngle2.setPosition(INTAKE_TWO_ANGLE_LOAD_POSITION);
                    intakePivot.setPosition(INTAKE_PIVOT_PASS_POSITION);


                    if (timerSeconds + 2.25 < currentTimeSeconds) {
                        currentMode = ELEVATORMODE;
                        timer = currentTime;
                        timerSeconds = currentTimeSeconds;
                    }

                    else if (timerSeconds + 2.0 < currentTimeSeconds) {
                        intakeClaw.setPosition(INTAKE_CLAW_OPEN_POSITION);

                    } else if (timerSeconds + 1.5 < currentTimeSeconds) {

                        slide1.setPosition(SLIDE_ONE_PASS_POSITION);
                        slide2.setPosition(SLIDE_TWO_PASS_POSITION);
                        outtakeClaw.setPosition(OUTTAKE_CLAW_CLOSED_POSITION);
                    } else if (timerSeconds + 1 < currentTimeSeconds) {

                        slide1.setPosition(SLIDE_ONE_PASS_POSITION);
                        slide2.setPosition(SLIDE_TWO_PASS_POSITION);

                    } else if(timerSeconds + .5 < currentTimeSeconds) {
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

                //after the pass has occured the block is locked into place so
                //that it is consistently dropped into the bucket when aligned
                //elevators are also sent to top so that the robot is ready to drop
                case ELEVATORMODE:
                    intakeAngle1.setPosition(INTAKE_ONE_ANGLE_LOAD_POSITION);
                    intakeAngle2.setPosition(INTAKE_TWO_ANGLE_LOAD_POSITION);
                    elevator1.setTargetPosition(HIGH_ELEVATOR_POSITION);
                    elevator2.setTargetPosition(HIGH_ELEVATOR_POSITION);
                    elevator1.setPower(1);
                    elevator2.setPower(1);
                    elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    outtakeClaw.setPosition(OUTTAKE_CLAW_CLOSED_POSITION);

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    if (timerSeconds + .8 < currentTimeSeconds) {
                        outtakeClaw.setPosition(OUTTAKE_CLAW_CLOSED_POSITION);
                        outtakeAngle.setPosition(OUTTAKE_ANGLE_DROP_POSITION);
                    } else {
                        outtakeClaw.setPosition(OUTTAKE_CLAW_PREDROP_POSITION);
                        outtakeAngle.setPosition(OUTTAKE_ANGLE_PREDROP_POSITION);
                    }

                    if (elevator1.getCurrentPosition() > HIGH_ELEVATOR_POSITION-50
                            && elevator2.getCurrentPosition() > HIGH_ELEVATOR_POSITION-50) {
                        currentMode = READYDROPMODE;
                    }

                    break;

                //position for when the block is loaded and ready to be placed,
                //it is not possible to change modes without first dropping the block
                //to prevent the block from ending up stuck
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

                //Robot controls lock and block drops, preventing the block
                //from missing the bucket
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

                //When the robot is not currently in a mode for handling blocks,
                //as such keeps the robot and claws in position to not be broken, e.g.
                //hitting walls
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




            //assign power to wheels
            frontLeftDrive.setPower(speed + strafe + turn);
            frontRightDrive.setPower(speed - strafe - turn);
            backLeftDrive.setPower(speed - strafe + turn);
            backRightDrive.setPower(speed + strafe - turn);


        }
    }
}
