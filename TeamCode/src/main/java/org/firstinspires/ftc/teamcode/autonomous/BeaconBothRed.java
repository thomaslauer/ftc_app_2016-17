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
@Autonomous(name = "Beacon Both Red")
public class BeaconBothRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LegolessRobot robot = new LegolessRobot();
        robot.initialize(hardwareMap, this);

        double liftMotorSpeed = 0.2;
        DcMotor liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double doorMotorSpeed = 0.08;
        DcMotor doorMotor = hardwareMap.dcMotor.get("doorMotor");
        doorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        robot.imu.update();
        robot.imu.resetStartPosition();

        robot.drive.driveAuto(-1.29, -1.29, robot.drive.speed);

        robot.thrower.setSpeed(0.4);
        while(robot.thrower.currentSpeed < robot.thrower.targetSpeed && opModeIsActive()) {
            robot.thrower.update();
        }
        sleep(1500);
        liftMotor.setPower(liftMotorSpeed);
        doorMotor.setPower(doorMotorSpeed);
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

        double distance = robot.ultrasonicSensor.getReadingAverage();

        if(!Double.isNaN(distance)) robot.drive.driveAuto(0, (18 - distance)/100.0, 750);

        double moveDistance = 0;

        if (robot.colorSensor.red() > robot.colorSensor.blue()) {
            robot.drive.driveAuto(-0.19, 0, (int) (robot.drive.speedSlow * 0.75));
            robot.drive.driveAuto(0, -0.19, (int) (robot.drive.speedSlow * 0.75f));
            sleep(200);
            robot.drive.driveAuto(0, 0.19, (int) (robot.drive.speedSlow * 0.75f));

            sleep(200);
            moveDistance = 0.19;
        } else if (robot.colorSensor.red() < robot.colorSensor.blue()) {
            robot.drive.driveAuto(0.225, 0, (int) (robot.drive.speedSlow * 0.75));
            robot.drive.driveAuto(0, -0.19, (int) (robot.drive.speedSlow * 0.75));
            sleep(200);
            robot.drive.driveAuto(0, 0.19, (int) (robot.drive.speedSlow * 0.75));
            sleep(200);
            moveDistance = -.225;
        }
        robot.imu.update();
        robot.drive.driveAuto(-1.33 + moveDistance,0,robot.drive.speed);

        distance = robot.ultrasonicSensor.getReadingAverage();

        if(!Double.isNaN(distance)) robot.drive.driveAuto(0, (18 - distance)/100.0, 750);

        if (robot.colorSensor.red() > robot.colorSensor.blue()) {
            robot.drive.driveAuto(-0.19, 0, robot.drive.speedSlow);
            robot.drive.driveAuto(0, -0.19, robot.drive.speedSlow);
            sleep(200);
            robot.drive.driveAuto(0, 0.19, robot.drive.speedSlow);

            sleep(200);
            moveDistance = 0.19;
        } else if (robot.colorSensor.red() < robot.colorSensor.blue()) {
            robot.drive.driveAuto(0.19, 0, robot.drive.speedSlow);
            robot.drive.driveAuto(0, -0.19, robot.drive.speedSlow);
            sleep(200);
            robot.drive.driveAuto(0, 0.19, robot.drive.speedSlow);
            sleep(200);
            moveDistance = -.225;
        }

//        if(robot.imu.getDeltaAngle() > 0) {
//            robot.drive.rotateClockwiseAuto(robot.imu.getDeltaAngle(), .25);
//        } else {
//            robot.drive.rotateCounterClockwiseAuto(-robot.imu.getDeltaAngle(), .25);
//        }
    }
}