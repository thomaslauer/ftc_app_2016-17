package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.configuration.LegolessRobot;

import java.text.DecimalFormat;

/**
 * Created by fello on 12/20/2016.
 */
@TeleOp(name="LEGOLESS TELEOP")
public class LegolessTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LegolessRobot robot = new LegolessRobot();
        robot.initialize(hardwareMap, this);

        waitForStart();

        DcMotor doorMotor = hardwareMap.dcMotor.get("doorMotor");

        double liftMotorSpeed = 0.25;
        DcMotor liftMotor = hardwareMap.dcMotor.get("liftMotor");

        while(opModeIsActive()) {

            if(gamepad1.a) {
                robot.imu.resetStartPosition();
            }

            double dpadSpeed = 0.75;

            if(gamepad1.b) dpadSpeed = 0.5;
            if(gamepad1.x) dpadSpeed = 1;

            double rotationSpeed = gamepad1.right_stick_x;

            if(gamepad1.right_bumper) rotationSpeed = dpadSpeed;
            if(gamepad1.left_bumper) rotationSpeed = -dpadSpeed;

            if(gamepad1.dpad_up) {
                robot.drive.drive(0, dpadSpeed, rotationSpeed);
            } else if (gamepad1.dpad_right) {
                robot.drive.drive(dpadSpeed, 0, rotationSpeed);
            } else if (gamepad1.dpad_down) {
                robot.drive.drive(0, -dpadSpeed, rotationSpeed);
            } else if (gamepad1.dpad_left) {
                robot.drive.drive(-dpadSpeed, 0, rotationSpeed);
            } else {
                robot.drive.smartDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                        rotationSpeed);
            }


            if(gamepad2.a) {
                robot.thrower.setSpeed(0.42);
            }
            if(gamepad2.b) {
                robot.thrower.setSpeed(0);
            }
            if(gamepad2.x) {
                robot.thrower.setSpeed(0.38);
            }
            if(gamepad2.y) {
                robot.thrower.setSpeed(0.45);
            }

            if(gamepad2.dpad_up) {
                robot.thrower.targetSpeed += robot.thrower.rampUpSpeed;
            } else if (gamepad2.dpad_down) {
                robot.thrower.targetSpeed -= robot.thrower.rampDownSpeed;
            }

            robot.thrower.update();

            DecimalFormat format = new DecimalFormat("#.00");

            telemetry.addData("Thrower Target", format.format(robot.thrower.targetSpeed * 100) + "%");
            telemetry.addData("Thrower Speed", format.format(robot.thrower.currentSpeed * 100) + "%");

            if(Math.abs(robot.thrower.currentSpeed - robot.thrower.targetSpeed) < 0.01) {
                telemetry.addData("Thrower Status", "!!!!READY!!!!");
            } else {
                telemetry.addData("Thrower Status", "not ready");
            }

            doorMotor.setPower(gamepad2.right_stick_y / 5);

            //robot.rightServo.setPosition(gamepad2.right_trigger);
            //robot.rightServo.setPosition(gamepad2.left_trigger);

            liftMotor.setPower(gamepad2.left_stick_y);
            telemetry.update();

            idle();
            robot.imu.update();

        }
    }
}
