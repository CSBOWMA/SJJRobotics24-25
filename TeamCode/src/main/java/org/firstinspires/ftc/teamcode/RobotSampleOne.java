package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.libraries.MovementCurves.MovementCurves;
import org.firstinspires.ftc.teamcode.libraries.robotPeripherals.*;


//Manual code, designed for one driver to grab samples
//and place them in the high bucket to score 8 points each
//The robot is handled in modes with search being the primary
//mode to find the block and match its orientation
@TeleOp
public class RobotSampleOne extends LinearOpMode {

    private enum Mode {

        DEFAULT,
        PUSH,
        SEARCH,
        GRAB,
        PASS,
        ELEVATOR,
        READYDROP,
        DROP;
    }


    public void runOpMode() throws InterruptedException {
        //initialize all robot positions, and relevant variables
        IMU  imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);
        imu.resetYaw();

        //handle timed events
        long timer = 0;
        double timerSeconds = 0;

        //change value to change the speed of joysticks
        final double TOTALSPEED = 1;

        //all relevant drive variables
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


        //peripherals of robot
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

        //thresholds for sensors so they can find blocks
        final int FRONTDISTANCETHRESHOLD = 40;
        final int BACKDISTANCETHRESHOLD = 35;

        //starts at default mode
        Mode currentMode = Mode.DEFAULT;

        long currentTime;
        double currentTimeSeconds;


        //initial robot position
        // required so robot fits size requirements
        //and peripherals do not collide
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
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //get current time in nanoseconds and seconds
            currentTime = System.nanoTime();
            currentTimeSeconds = currentTime/1_000_000_000.0;

            //calculate what direction the robot should drive
            speed = -gamepad1.left_stick_y*TOTALSPEED;
            strafe = gamepad1.left_stick_x*TOTALSPEED;
            turn = gamepad1.right_stick_x*TOTALSPEED;

            //Input sets the mode
            if (gamepad1.triangle && currentMode != Mode.READYDROP) {
                currentMode = Mode.DEFAULT;
            }

            if (gamepad1.square&& currentMode != Mode.READYDROP) {
                currentMode = Mode.GRAB;
                timer = 2*currentTime;
                timerSeconds = 2*currentTimeSeconds;
            }

            if (gamepad1.a && currentMode != Mode.READYDROP) {
                currentMode = Mode.SEARCH;
                timer = currentTime;
                timerSeconds = currentTimeSeconds;
            }
            //keep track of relevant data
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

                //      if (gamepad1.right_trigger > 0.2) {
                //          slide1.setPosition(slide1.getPosition() + 0.0025);//.decrease();
                //          slide2.setPosition(slide2.getPosition() - 0.0025);//.increase();
                //      }
                //      if (gamepad1.left_trigger > 0.2) {
                //          slide1.setPosition(slide1.getPosition() - 0.0025);//.increase();
                //          slide2.setPosition(slide2.getPosition() + 0.0025);//.decrease();
                //      }
                //      frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                //      backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                //      frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                //      frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                //      intakeAngle1.setPosition(INTAKE_ONE_ANGLE_GRAB_POSITION);
                //      intakeAngle2.setPosition(INTAKE_TWO_ANGLE_GRAB_POSITION);

                //      if (!gamepad1.square) {
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
                case SEARCH:

                    speed *= .3;
                    strafe *= .4;
                    turn *= .5;

                    if (gamepad1.right_trigger > 0.2) {
                        intakeSlide.setPosition(intakeSlide.getPosition()+.005);

                    }
                    if (gamepad1.left_trigger > 0.2) {
                        intakeSlide.setPosition(intakeSlide.getPosition()-.005);
                    }

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                    //if block is found go into grab mode
                    if (frontSensor.getDistance(DistanceUnit.MM) < FRONTDISTANCETHRESHOLD
                            && backSensor.getDistance(DistanceUnit.MM) < BACKDISTANCETHRESHOLD) {
                        intakePivot.setPosition(intakePivot.getPosition());
                        intakeAngle.grab();
                        currentMode = Mode.GRAB;
                        timer = currentTime*2;
                        timerSeconds = currentTimeSeconds*2;
                    }

                    //oscillate pivot in the case that the block is not aligned
                    intakePivot.setPosition(MovementCurves.linear(
                            ((double)(currentTime%2_500_000_000L))/2_500_000_000.0D));

                    //all states the robot should be in when in this mode
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

                //initialized after finding a block or override button is pressed
                //in this mode the claw lowers and stops moving, allowing user to grab block
                case GRAB:

                    speed *= .3;
                    strafe *= .4;
                    turn *= .5;

                    if (gamepad1.right_trigger > 0.2) {
                        intakeSlide.setPosition(intakeSlide.getPosition()+.005);
                    }
                    if (gamepad1.left_trigger > 0.2) {
                        intakeSlide.setPosition(intakeSlide.getPosition()-.005);
                    }
                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    intakeAngle.grab();

                    //grabs block and gets ready to load
                    if(gamepad1.right_bumper) {
                        intakeClaw.close();
                        timer = currentTime;
                        timerSeconds = currentTimeSeconds;
                    }

                    if (timerSeconds + .5 < currentTimeSeconds) {
                        currentMode = Mode.PASS;
                        timer = currentTime;
                        timerSeconds = currentTimeSeconds;
                    } //else if (timerSeconds + .5 < currentTimeSeconds) {
                    //  currentMode = SEARCHMODE;
                    //}

                    elevator.bottom();
                    outtakeAngle.load();
                    outtakeClaw.open();
                    break;

                // Block is now grabbed and ready to be passed through the robot,
                //consistent issue is servo alignment, will need to be fixed in future
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

                //after the pass has occurred the block is locked into place so
                //that it is consistently dropped into the bucket when aligned
                //elevators are also sent to top so that the robot is ready to drop
                case ELEVATOR:
                    intakeAngle.load();
                    elevator.top();

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    if (timerSeconds + .8 < currentTimeSeconds) {
                        outtakeClaw.close();
                        outtakeAngle.drop();
                    } else {
                        outtakeClaw.predrop();
                        outtakeAngle.predrop();
                    }

                    if (elevator.getPosition() > .99) {
                        currentMode = Mode.READYDROP;
                    }

                    break;

                //position for when the block is loaded and ready to be placed,
                //it is not possible to change modes without first dropping the block
                //to prevent the block from ending up stuck
                case READYDROP:

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    if (gamepad1.circle) {
                        currentMode = Mode.DROP;
                        timer = currentTime;
                        timerSeconds = currentTimeSeconds;
                    } else {

                        intakeAngle.load();
                        elevator.top();
                        outtakeAngle.drop();
                        outtakeClaw.close();
                    }
                    break;

                //Robot controls lock and block drops, preventing the block
                //from missing the bucket
                case DROP:
                    outtakeClaw.open();

                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    speed = 0;
                    strafe = 0;
                    turn = 0;
                    if (timerSeconds + .75 < currentTimeSeconds) {
                        currentMode = Mode.DEFAULT;
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
                    intakePivot.pass();
                    intakeClaw.open();
                    intakeAngle.load();
                    outtakeAngle.load();
                    outtakeClaw.open();
                    elevator.bottom();
                    intakeSlide.prepass();
            }




            //assign power to wheels
            frontLeftDrive.setPower(speed + strafe + turn);
            frontRightDrive.setPower(speed - strafe - turn);
            backLeftDrive.setPower(speed - strafe + turn);
            backRightDrive.setPower(speed + strafe - turn);


        }
    }
}
