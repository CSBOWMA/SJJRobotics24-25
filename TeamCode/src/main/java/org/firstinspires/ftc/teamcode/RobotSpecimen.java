package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.libraries.robotPeripherals.*;
import org.firstinspires.ftc.teamcode.libraries.movementCurves.MovementCurves;


//Manual code, designed for two drivers to grab samples
//then bring them to the zone to create specimen to finally
//place them on the high rung to score 10 points each
//The robot is handled in modes with search being the primary
//mode to find the block and match its orientation
@TeleOp
public class RobotSpecimen extends LinearOpMode {

    private enum Mode {

        DEFAULT,
        PREPLACE,
        SEARCH,
        GRAB,
        PASS,
        ELEVATOR,
        READYHANG,
        HANG,
        POSTHANG;
    }

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


        OuttakeAngle outtakeAngle = new OuttakeAngle(hardwareMap);
        OuttakeClaw outtakeClaw = new OuttakeClaw(hardwareMap);

        IntakeAngle intakeAngle = new IntakeAngle(hardwareMap);
        IntakeClaw intakeClaw = new IntakeClaw(hardwareMap);
        IntakePivot intakePivot = new IntakePivot(hardwareMap);
        IntakeSlide intakeSlide = new IntakeSlide(hardwareMap);


        Elevator elevator = new Elevator(hardwareMap);

        RevColorSensorV3 frontSensor;
        RevColorSensorV3 backSensor;

        frontSensor = hardwareMap.get(RevColorSensorV3.class, "c1");
        backSensor = hardwareMap.get(RevColorSensorV3.class, "c2");

        final int FRONTDISTANCETHRESHOLD = 40;
        final int BACKDISTANCETHRESHOLD = 35;

        Mode currentMode = Mode.DEFAULT;

        long currentTime;
        double currentTimeSeconds;

        outtakeAngle.drop();
        outtakeClaw.close();
        timer = System.nanoTime();
        intakeSlide.close();
        while(timer + 1_000_000_000 > System.nanoTime());
        intakeAngle.load();
        intakeClaw.close();
        outtakeAngle.load();
        intakePivot.pass();

        waitForStart();
        while (opModeIsActive()) {
            //all drive control, joysticks
            currentTime = System.nanoTime();
            currentTimeSeconds = currentTime/1_000_000_000.0;

            speed = -gamepad1.left_stick_y*TOTALSPEED;
            strafe = gamepad1.left_stick_x*TOTALSPEED;
            turn = gamepad1.right_stick_x*TOTALSPEED;

            if (gamepad2.triangle && currentMode != Mode.READYHANG) {
                currentMode = Mode.DEFAULT;
            }
            if (gamepad2.square && currentMode != Mode.READYHANG) {
                currentMode = Mode.PREPLACE;
                timer = currentTime;
                timerSeconds = currentTimeSeconds;
            }
            if (gamepad2.a && currentMode != Mode.READYHANG) {
                currentMode = Mode.SEARCH;
                timer = currentTime;
                timerSeconds = currentTimeSeconds;
            }

            telemetry.addData("currentMode", currentMode);
            telemetry.addData("sens1D", frontSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("sens2D", backSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("currentSeconds", currentTimeSeconds);
            telemetry.addData("seconds", timerSeconds);

            telemetry.update();

            switch (currentMode) {

                case SEARCH:

                    speed *= .3;
                    strafe *= .3;
                    turn *= .3;

                    if (gamepad2.right_trigger > 0.2) {
                        intakeSlide.setPosition(intakeSlide.getPosition()+.005);

                    }
                    if (gamepad2.left_trigger > 0.2) {
                        intakeSlide.setPosition(intakeSlide.getPosition()-.005);
                    }


                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


                    if (frontSensor.getDistance(DistanceUnit.MM) < FRONTDISTANCETHRESHOLD
                            && backSensor.getDistance(DistanceUnit.MM) < BACKDISTANCETHRESHOLD) {
                        intakePivot.setPosition(intakePivot.getPosition());
                        intakeAngle.grab();
                        currentMode = Mode.GRAB;
                        timer = currentTime;
                        timerSeconds = currentTimeSeconds;
                    }


                    intakePivot.setPosition( MovementCurves.linear(
                            ((double)(currentTime%2_500_000_000L))/2_500_000_000.0D));
                    telemetry.addData("SWITCH", (currentTimeSeconds*2)%5);

                    elevator.bottom();
                    outtakeAngle.load();
                    outtakeClaw.open();
                    intakeClaw.open();
                    intakeAngle.search();

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    break;
                case PREPLACE:

                    speed *= .3;
                    strafe *= .3;
                    turn *= .3;

                    if (gamepad2.right_trigger > 0.2) {
                        intakeSlide.setPosition(intakeSlide.getPosition()+.005);

                    }
                    if (gamepad2.left_trigger > 0.2) {
                        intakeSlide.setPosition(intakeSlide.getPosition()-.005);
                    }


                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


                    if (frontSensor.getDistance(DistanceUnit.MM) < FRONTDISTANCETHRESHOLD
                            && backSensor.getDistance(DistanceUnit.MM) < BACKDISTANCETHRESHOLD) {
                        intakePivot.setPosition(intakePivot.getPosition());
                        intakeAngle.grab();
                        currentMode = Mode.GRAB;
                        timer = currentTime;
                        timerSeconds = currentTimeSeconds;
                    }

                    intakePivot.pass();
                    telemetry.addData("SWITCH", (currentTimeSeconds*2)%5);

                    elevator.bottom();
                    outtakeAngle.load();
                    outtakeClaw.open();
                    intakeClaw.open();
                    intakeAngle.search();

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    break;
                case GRAB:
                    speed = 0;
                    strafe = 0;
                    turn = 0;

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    intakeAngle.grab();


                    if(timerSeconds + .25 < currentTimeSeconds) {
                        intakeClaw.close();
                    }

                    if (timerSeconds + .5 < currentTimeSeconds) {
                        currentMode = Mode.PASS;
                        timer = currentTime;
                        timerSeconds = currentTimeSeconds;
                    }// else if (timerSeconds + .5 < currentTimeSeconds) {
                    //   currentMode = SEARCHMODE;
                    //}

                    elevator.bottom();
                    outtakeAngle.load();
                    outtakeClaw.open();
                    break;

                case PASS:

                    //initial positions, lets driver move full speed so they may get ready to place
                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    intakeAngle.load();
                    intakePivot.pass();

                    if (timerSeconds + 2.25 < currentTimeSeconds) {
                        currentMode = Mode.ELEVATOR;
                        timer = currentTime;
                        timerSeconds = currentTimeSeconds;
                    }

                    else if (timerSeconds + 2.0 < currentTimeSeconds) {
                        intakeClaw.open();
                    } else if (timerSeconds + 1.5 < currentTimeSeconds) {
                        intakeSlide.close();
                        outtakeClaw.close();
                    } else if (timerSeconds + 1 < currentTimeSeconds) {
                        intakeSlide.close();
                    } else if(timerSeconds + .5 < currentTimeSeconds) {
                        intakeSlide.prepass();
                        outtakeAngle.load();
                        outtakeClaw.open();
                        elevator.bottom();
                    } else {
                        intakeSlide.prepass();
                    }
                    break;
                case ELEVATOR:
                    intakeAngle.load();
                    elevator.hang();
                    outtakeAngle.predrop();
                    outtakeClaw.close();

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                    if (elevator.getPosition() > .43) {
                        currentMode = Mode.READYHANG;
                    }

                    break;
                case READYHANG:

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    if (gamepad2.circle) {
                        currentMode = Mode.HANG;
                        timer = currentTime;
                        timerSeconds = currentTimeSeconds;
                    } else {

                        intakeAngle.load();

                        elevator.hang();
                        outtakeAngle.predrop();
                        outtakeClaw.predrop();
                    }
                    break;

                case HANG:

                    speed *= .3;
                    strafe *= .3;
                    turn *= .3;
                   frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    speed = 0;
                    strafe = 0;
                    turn = 0;
                    currentMode = Mode.POSTHANG;
                    timerSeconds = currentTimeSeconds;
               //     if (timerSeconds + 1 < currentTimeSeconds) {
               //         currentMode = POSTHANGMODE;
               //         timerSeconds = currentTimeSeconds;
               //     } else if (timerSeconds + .5 < currentTimeSeconds) {
               //         outtakeClaw.setPosition(OUTTAKE_CLAW_OPEN_POSITION);
               //     }
                    break;

                case POSTHANG:

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    intakePivot.pass();
                    intakeClaw.open();
                    intakeAngle.load();
                    outtakeAngle.postdrop();
                    outtakeClaw.open();
                    intakeSlide.prepass();

                    if (timerSeconds + 3 < currentTimeSeconds) {
                        currentMode = Mode.DEFAULT;
                    }

                    break;
                default:

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                    elevator.bottom();

                    intakePivot.pass();
                    intakeClaw.open();
                    intakeAngle.load();
                    outtakeAngle.load();
                    outtakeClaw.open();
                    intakeSlide.prepass();
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
