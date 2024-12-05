package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.DriveMainAuto;

@Autonomous
public class NewAuto extends LinearOpMode implements DriveMainAuto {

    public double rotationLeft = 0;
    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //send data to console
        telemetry.addData("Status", "Initialized");

        //load motors
        loadMotors(hardwareMap, new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        //send update data
        telemetry.update();
        waitForStart();

        while(facing(90)){
            telemetry.addData("hello", "1");
            telemetry.addData("hello222", rotationLeft);
            telemetry.addData("cur", (int)imu.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+180);
            telemetry.addData("frontLeftDrive", frontLeftDrive.getMotor().getPower());
            telemetry.addData("frontRightDrive", frontRightDrive.getMotor().getPower());
            telemetry.addData("backRightDrive", backRightDrive.getMotor().getPower());
            telemetry.addData("backLeftDrive", backLeftDrive.getMotor().getPower());


            telemetry.update();
            drive();
        }
        rotationLeft=0;
        drive();
        waitt(1, runtime);

        while(facing(45)){
            telemetry.addData("hello", "2");
            telemetry.addData("hello222", rotationLeft);
            telemetry.addData("cur", (int)imu.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+180);
            telemetry.addData("frontLeftDrive", frontLeftDrive.getMotor().getPower());
            telemetry.addData("frontRightDrive", frontRightDrive.getMotor().getPower());
            telemetry.addData("backRightDrive", backRightDrive.getMotor().getPower());
            telemetry.addData("backLeftDrive", backLeftDrive.getMotor().getPower());


            telemetry.update();
            drive();
        }
        rotationLeft=0;
        drive();
        waitt(1, runtime);


        while(facing(270)){
            telemetry.addData("hello", "3");
            telemetry.addData("hello222", rotationLeft);
            telemetry.addData("cur", (int)imu.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+180);
            telemetry.addData("frontLeftDrive", frontLeftDrive.getMotor().getPower());
            telemetry.addData("frontRightDrive", frontRightDrive.getMotor().getPower());
            telemetry.addData("backRightDrive", backRightDrive.getMotor().getPower());
            telemetry.addData("backLeftDrive", backLeftDrive.getMotor().getPower());


            telemetry.update();
            drive();
        }
        rotationLeft=0;
        drive();
        waitt(1, runtime);



        while(facing(0)){
            drive();
        }

    }
    public boolean facing(int target){
        rotationLeft=0;
        boolean correctDriction=false;
        double power = 0.05;
        int cur = (int)imu.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+180;
        if (!CheckDriveStraight.isWithinTolerance(cur, target, 5)){
            rotationLeft+=power/4;
            correctDriction=true;
        }
        if (!CheckDriveStraight.isWithinTolerance(cur, target, 20)){
            rotationLeft+=power/4;
        }
        if (!CheckDriveStraight.isWithinTolerance(cur, target, 40)){
            rotationLeft+=power/4;
        }
        if (!CheckDriveStraight.isWithinTolerance(cur, target, 80)){
            rotationLeft+=power/4;
        }
        return correctDriction;
    }

    public void drive() {
        backLeftDrive.setPower(-rotationLeft); //backR
        backRightDrive.setPower(+rotationLeft); //frontL
        frontLeftDrive.setPower(-rotationLeft);  //frontR
        frontRightDrive.setPower(+rotationLeft);
        telemetry.addData("drive", gamepad1.left_stick_y);

    }

    private void waitt(double sec, ElapsedTime runtime){
        runtime.reset();
        while(runtime.seconds() < sec){

        }
    }


}

