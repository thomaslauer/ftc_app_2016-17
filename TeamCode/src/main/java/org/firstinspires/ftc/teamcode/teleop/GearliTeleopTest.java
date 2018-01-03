package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by fello on 9/20/2016.
 */
@TeleOp(name="Gearli")
public class GearliTeleopTest extends LinearOpMode {

    DcMotor rightMotor1;
    DcMotor leftMotor1;
    DcMotor rightMotor2;
    DcMotor leftMotor2;


    public void runOpMode() throws InterruptedException {
        rightMotor1 = hardwareMap.dcMotor.get("rightMotor1");
        leftMotor1  = hardwareMap.dcMotor.get("leftMotor1");
        rightMotor2 = hardwareMap.dcMotor.get("rightMotor2");
        leftMotor2  = hardwareMap.dcMotor.get("leftMotor2");

        waitForStart();

        while (opModeIsActive()) {
            rightMotor1.setPower(gamepad1.right_stick_y);
            rightMotor2.setPower(gamepad1.right_stick_y);
            leftMotor1.setPower(-gamepad1.left_stick_y);
            leftMotor2.setPower(-gamepad1.left_stick_y);
        }
    }
}