package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
/**
 * Created by fello on 2/14/2017.
 */
@TeleOp(name="GearliThrower")
public class GearliThrower extends LinearOpMode {

    DcMotor rightMotor;
    DcMotor leftMotor;


    public void runOpMode() throws InterruptedException {
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor  = hardwareMap.dcMotor.get("leftMotor");


        waitForStart();

        while (opModeIsActive()) {
            rightMotor.setPower(gamepad1.right_stick_y);
            leftMotor.setPower(-gamepad1.left_stick_y);

        }
    }
}