package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.configuration.HolonomicRobot;

/**
 * Created by Thomas on 9/11/2016.
 */
@TeleOp(name="Holonomic Teleop")
public class HolonomicTeleop extends LinearOpMode {

    HolonomicRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HolonomicRobot();
        robot.initialize(hardwareMap, this);
        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.a) {
                robot.imu.resetStartPosition();
            }

            double speed = 0.5;

            if(gamepad1.right_bumper) speed = 0.75;

            if(gamepad1.dpad_up) {
                robot.drive.drive(speed, 0, gamepad1.right_stick_x * speed);
            } else if(gamepad1.dpad_down) {
                robot.drive.drive(-speed, 0, gamepad1.right_stick_x * speed);
            } else if(gamepad1.dpad_right) {
                robot.drive.drive(0, -speed, gamepad1.right_stick_x * speed);
            } else if(gamepad1.dpad_left) {
                robot.drive.drive(0, speed, gamepad1.right_stick_x * speed);
            } else {
                robot.drive.smartDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                        gamepad1.right_stick_x);
            }
        }
    }
}
