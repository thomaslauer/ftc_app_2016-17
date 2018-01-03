package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.configuration.LegolessRobot;

/**
 * Created by fello on 3/4/2017.
 */
@Autonomous(name = "v2 Beacon Both Blue")
public class BeaconBothBlue2 extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        LegolessRobot robot = new LegolessRobot();
        robot.initialize(hardwareMap, this);

        double liftMotorSpeed = 0.15;
        DcMotor liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double doorMotorSpeed = 0.08;
        DcMotor doorMotor = hardwareMap.dcMotor.get("doorMotor");

        double maxError = 0.087;

        telemetry.addData("Status", "READY");
        telemetry.update();

        waitForStart();

        robot.imu.update();
        robot.imu.resetStartPosition();

        robot.drive.driveAuto(1.06, -1.06, robot.drive.speedFast);

        robot.ne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.se.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.sw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.nw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (robot.distanceSensor.getLightDetected() < 0.5 && opModeIsActive()){
            robot.ne.setPower(0.3);
            robot.se.setPower(-0.3);
            robot.nw.setPower(0.3);
            robot.sw.setPower(-0.3);
            telemetry.addData("Light value", robot.distanceSensor.getLightDetected());
            telemetry.update();
            idle();
        }

        robot.ne.setPower(0);
        robot.se.setPower(0);
        robot.nw.setPower(0);
        robot.sw.setPower(0);

        double distance = robot.ultrasonicSensor.getReadingAverage();

        if(!Double.isNaN(distance)) robot.drive.driveAuto(0, (18 - distance)/100.0, 750);

        robot.imu.update();
        telemetry.addData("IMU ANGLE", robot.imu.getDeltaAngle());
        telemetry.update();
        if(Math.abs(robot.imu.getDeltaAngle())> maxError) {robot.drive.rotateAuto(-robot.imu.getDeltaAngle(), 0.2);}

        robot.thrower.setSpeed(0.42);
        while(robot.thrower.currentSpeed < robot.thrower.targetSpeed && opModeIsActive()) {
            robot.thrower.update();
        }

        sleep(1000);
        liftMotor.setPower(liftMotorSpeed);
        sleep(4500);

        robot.thrower.setSpeed(0);
        while(robot.thrower.currentSpeed > robot.thrower.targetSpeed && opModeIsActive()) {
            robot.thrower.update();
        }
        liftMotor.setPower(0);
        doorMotor.setPower(0);

        double nextMoveAddition = 0;
        if (robot.colorSensor.red() < robot.colorSensor.blue()) {
            robot.drive.driveAuto(-0.19, 0, robot.drive.speedSlow);
            robot.drive.driveAuto(0, -0.16, robot.drive.speedSlow);
            sleep(200);
            robot.drive.driveAuto(0, 0.16, robot.drive.speedSlow);

            sleep(200);
            nextMoveAddition = 0.19;
        } else if (robot.colorSensor.red() > robot.colorSensor.blue()) {
            robot.drive.driveAuto(0.21, 0, robot.drive.speedSlow);
            robot.drive.driveAuto(0, -0.16, robot.drive.speedSlow);
            sleep(200);
            robot.drive.driveAuto(0, 0.16, robot.drive.speedSlow);
            sleep(200);
            nextMoveAddition = -.21;
        }

        robot.imu.update();
        telemetry.addData("IMU ANGLE", robot.imu.getDeltaAngle());
        telemetry.update();
        if(Math.abs(robot.imu.getDeltaAngle())> maxError) {robot.drive.rotateAuto(-robot.imu.getDeltaAngle(), 0.2);}

        robot.drive.driveAuto(1.18 + nextMoveAddition, 0, robot.drive.speedFast);

        robot.ne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.se.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.sw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.nw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (robot.distanceSensor.getLightDetected() < 0.5 && opModeIsActive()){
            robot.ne.setPower(0.3);
            robot.se.setPower(-0.3);
            robot.nw.setPower(0.3);
            robot.sw.setPower(-0.3);
            telemetry.addData("Light value", robot.distanceSensor.getLightDetected());
            telemetry.update();
            idle();
        }

        robot.ne.setPower(0);
        robot.se.setPower(0);
        robot.nw.setPower(0);
        robot.sw.setPower(0);

        robot.imu.update();
        telemetry.addData("IMU ANGLE", robot.imu.getDeltaAngle());
        telemetry.update();
        if(Math.abs(robot.imu.getDeltaAngle())> maxError) {robot.drive.rotateAuto(-robot.imu.getDeltaAngle(), 0.2);}

        distance = robot.ultrasonicSensor.getReadingAverage();
        if(!Double.isNaN(distance)) robot.drive.driveAuto(0, (18 - distance)/100.0, 750);

        if (robot.colorSensor.red() < robot.colorSensor.blue()) {
            robot.drive.driveAuto(-0.19, 0, robot.drive.speedSlow);
            robot.drive.driveAuto(0, -0.17, robot.drive.speedSlow);
            sleep(200);
//            robot.drive.driveAuto(0, 0.16, robot.drive.speedSlow);

//            sleep(200);
            //nextMoveAddition = 0.17;
        } else if (robot.colorSensor.red() > robot.colorSensor.blue()) {
            robot.drive.driveAuto(0.21, 0, robot.drive.speedSlow);
            robot.drive.driveAuto(0, -0.17, robot.drive.speedSlow);
            sleep(200);
            //robot.drive.driveAuto(0, 0.17, robot.drive.speedSlow);
            //sleep(200);
            //nextMoveAddition = -.21;
        }

        robot.drive.driveAuto(0, 0.25, robot.drive.speed);
    }
}
