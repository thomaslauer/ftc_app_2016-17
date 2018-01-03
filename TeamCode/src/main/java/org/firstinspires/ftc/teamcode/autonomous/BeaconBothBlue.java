package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.teamcode.hardware.configuration.LegolessRobot;

@Autonomous(name = "Beacon Blue Both")
public class BeaconBothBlue extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        LegolessRobot robot = new LegolessRobot();
        robot.initialize(hardwareMap, this);

        double liftMotorSpeed = 0.15;
        DcMotor liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double doorMotorSpeed = 0.08;
        DcMotor doorMotor = hardwareMap.dcMotor.get("doorMotor");

        UltrasonicSensor ultrasonicSensor = hardwareMap.ultrasonicSensor.get("ultrasonicSensor");

        telemetry.addData("Status", "READY");

        waitForStart();

        robot.imu.update();
        robot.imu.resetStartPosition();

        robot.drive.driveAuto(1.29, -1.29, robot.drive.speed);

        //doorMotor.setPower(doorMotorSpeed);

        robot.thrower.setSpeed(0.40);
        while(robot.thrower.currentSpeed < robot.thrower.targetSpeed && opModeIsActive()) {
            robot.thrower.update();
        }

        sleep(2000);
        liftMotor.setPower(liftMotorSpeed);
        sleep(4500);

        robot.thrower.setSpeed(0);
        while(robot.thrower.currentSpeed > robot.thrower.targetSpeed && opModeIsActive()) {
            robot.thrower.update();
        }
        liftMotor.setPower(0);
        doorMotor.setPower(0);

        robot.imu.update();
        Log.d("IMU", "" + robot.imu.getDeltaAngle());
        double startAngle = robot.imu.getDeltaAngle();
        ultrasonicSensor.getUltrasonicLevel(); // dummy read to reset us sensor
        telemetry.addData("red", robot.colorSensor.red());
        telemetry.addData("blue", robot.colorSensor.blue());
        telemetry.addData("us distance", ultrasonicSensor.getUltrasonicLevel());
        telemetry.update();


        double distance = robot.ultrasonicSensor.getReadingAverage();

        if(!Double.isNaN(distance)) robot.drive.driveAuto(0, (18 - distance)/100.0, 750);

        double nextMoveAddition = 0;

        if (robot.colorSensor.red() < robot.colorSensor.blue()) {
            robot.drive.driveAuto(-0.19, 0, robot.drive.speedSlow);
            robot.drive.driveAuto(0, -0.19, robot.drive.speedSlow);
            sleep(200);
            robot.drive.driveAuto(0, 0.19, robot.drive.speedSlow);

            sleep(200);
            nextMoveAddition = 0.19;
        } else if (robot.colorSensor.red() > robot.colorSensor.blue()) {
            robot.drive.driveAuto(0.24, 0, robot.drive.speedSlow);
            robot.drive.driveAuto(0, -0.19, robot.drive.speedSlow);
            sleep(200);
            robot.drive.driveAuto(0, 0.19, robot.drive.speedSlow);
            sleep(200);
            nextMoveAddition = -.24;
        }

        robot.drive.driveAuto(1.33 + nextMoveAddition, 0, robot.drive.speed);

        telemetry.addData("red", robot.colorSensor.red());
        telemetry.addData("blue", robot.colorSensor.blue());
        telemetry.addData("us distance", ultrasonicSensor.getUltrasonicLevel());
        telemetry.update();


        distance = robot.ultrasonicSensor.getReadingAverage();
        if(!Double.isNaN(distance)) robot.drive.driveAuto(0, (18 - distance)/100.0, 750);

        if (robot.colorSensor.red() < robot.colorSensor.blue()) {
            robot.drive.driveAuto(-0.19, 0, robot.drive.speedSlow);
            robot.drive.driveAuto(0, -0.19, robot.drive.speedSlow);
            sleep(200);
            robot.drive.driveAuto(0, 0.19, robot.drive.speedSlow);

            sleep(200);
            //nextMoveAddition = 0.19;
        } else if (robot.colorSensor.red() > robot.colorSensor.blue()) {
            robot.drive.driveAuto(0.24, 0, robot.drive.speedSlow);
            robot.drive.driveAuto(0, -0.19, robot.drive.speedSlow);
            sleep(200);
            robot.drive.driveAuto(0, 0.19, robot.drive.speedSlow);
            sleep(200);
            //nextMoveAddition = -.24;
        }
    }
}
