package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.libraries.AutoRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous
public class RobotClassAuto extends LinearOpMode  {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        AutoRobot robot = new AutoRobot(hardwareMap, telemetry);
        robot.initialPosition();
        waitForStart();

        robot.getImu().resetYaw(); //if you move the robot at all between init and running
        //yaw will be incorrect, this handles that
        telemetry.addData("Status", "Running");
        telemetry.update();


        //preload block in bucket
        robot.elevatorTop();
        robot.outtakeAngleDrop();
        robot.outtakeClawClose();
        robot.driveLeftInchesIMU(6);
        robot.face(0);
        robot.driveBackwardsInchesIMU(12);
        robot.face(45);
        robot.dropSampleAndReset();

        //proceed to go for second block
        robot.face(90);
        robot.waitSeconds(.2);
        robot.driveRightInchesIMU(8.2);
        robot.driveForwardsInchesIMU(13.2);
        robot.grabAndLoadSample();

        //after first block grab go for bucket
        robot.driveBackwardsInchesIMU(14.1);
        robot.driveLeftInchesIMU(9);
        robot.waitSeconds(.5);
        robot.face(45);
        robot.waitSeconds(.5);
        robot.dropSampleAndReset();
        robot.face(90-5);
        robot.waitSeconds(.5);

        //go for second block
        robot.driveLeftInchesIMU(2.4);
        robot.driveForwardsInchesIMU(13.3);
        robot.grabAndLoadSample();
        robot.driveBackwardsInchesIMU(10);
        robot.waitSeconds(.75);

        //line up with bucket for drop
        robot.face(45);
        robot.driveBackwardsInchesIMU(2);
        robot.dropSampleAndReset();
        robot.elevatorBottom();
        robot.waitSeconds(2);

        //end of autonomous


    }
}