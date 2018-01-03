package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.configuration.LegolessRobot;

/**
 * Created by fello on 1/11/2017.
 */
@Autonomous(name = "Red Beacon")
public class BeaconRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LegolessRobot robot = new LegolessRobot();
        robot.initialize(hardwareMap, this);

        double liftMotorSpeed = 0.125;
        DcMotor liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double doorMotorSpeed = 0.08;
        DcMotor doorMotor = hardwareMap.dcMotor.get("doorMotor");

        waitForStart();

        robot.imu.update();
        robot.imu.resetStartPosition();

        robot.drive.driveAuto(-1.29, -1.29, 1500);

        //1robot.drive.rotateCounterClockwiseAuto(10.0 * Math.PI / 180.0, 0.5);

//        doorMotor.setPower(doorMotorSpeed);
//
//        robot.thrower.setSpeed(0.42);
//        while(robot.thrower.currentSpeed < robot.thrower.targetSpeed && opModeIsActive()) {
//            robot.thrower.update();
//        }
//        sleep(1500);
//        liftMotor.setPower(liftMotorSpeed);
//        sleep(3000);
//        liftMotor.setPower(liftMotorSpeed * 2);
//        sleep(3000);
//
//        robot.thrower.setSpeed(0);
//        while(robot.thrower.currentSpeed > robot.thrower.targetSpeed && opModeIsActive()) {
//            robot.thrower.update();
//        }
//        liftMotor.setPower(0);
//        doorMotor.setPower(0);

        robot.imu.update();
        Log.d("IMU", "" + robot.imu.getDeltaAngle());
        double startAngle = robot.imu.getDeltaAngle();
        if (robot.colorSensor.red() > robot.colorSensor.blue()) {
            robot.drive.driveAuto(-0.19, 0, 1000);
            robot.drive.driveAuto(0, -0.15, 1000);
            sleep(200);
            robot.drive.driveAuto(0, 0.19, 1000);

            sleep(200);
            robot.drive.driveAuto(.14, 0, 1000);
        } else {
            robot.drive.driveAuto(0.225, 0, 1000);
            robot.drive.driveAuto(0, -0.15, 1000);
            sleep(200);
            robot.drive.driveAuto(0, 0.15 ,1000);
            sleep(200);
            robot.drive.driveAuto(-.225,0,1000);
        }
        robot.imu.update();
        Log.d("IMU", "" + (startAngle - robot.imu.getDeltaAngle()));
    }
}
