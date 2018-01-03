package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.configuration.LegolessRobot;

/**
 * Created by fello on 1/11/2017.
 */
@Autonomous(name = "Blue Beacon")
public class BeaconBlue extends LinearOpMode {
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

        robot.drive.driveAuto(1.4, -1.4, 1500);

        doorMotor.setPower(doorMotorSpeed);

        robot.thrower.setSpeed(0.42);
        while(robot.thrower.currentSpeed < robot.thrower.targetSpeed && opModeIsActive()) {
            robot.thrower.update();
        }
        sleep(1500);
        liftMotor.setPower(liftMotorSpeed);
        sleep(3000);
        liftMotor.setPower(liftMotorSpeed * 2);
        sleep(3000);

        robot.thrower.setSpeed(0);
        while(robot.thrower.currentSpeed < robot.thrower.targetSpeed && opModeIsActive()) {
            robot.thrower.update();
        }
        liftMotor.setPower(0);
        doorMotor.setPower(0);

        if (robot.colorSensor.red() > robot.colorSensor.blue()){
            robot.drive.driveAuto(0.125 , 0 , 1000);
            robot.drive.driveAuto(0 , -0.15 , 1000);
            sleep(200);
            robot.drive.driveAuto(0 , 0.15 , 1000);

            sleep(200);
            robot.drive.driveAuto(-.125,0,1000);
        } else     {
            robot.drive.driveAuto(-0.225 , 0 , 1000);
            robot.drive.driveAuto(0 , -0.15 , 1000);
            sleep(200);
            robot.drive.driveAuto(0 , 0.15 , 1000);
            sleep(200);
            robot.drive.driveAuto(.225,0,1000);
        }

//        if(robot.imu.getDeltaAngle() > 0) {
//            robot.drive.rotateClockwiseAuto(robot.imu.getDeltaAngle(), .25);
//        } else {
//            robot.drive.rotateCounterClockwiseAuto(-robot.imu.getDeltaAngle(), .25);
//        }
    }
}
