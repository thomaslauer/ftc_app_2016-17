package org.firstinspires.ftc.teamcode.testing.thrower;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by fello on 10/16/2016.
 */
@TeleOp(name= "Multi Power Thrower Test")
public class MultiPowerThowerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor rightMotor = hardwareMap.dcMotor.get("rightMotor");
        DcMotor leftMotor = hardwareMap.dcMotor.get("leftMotor");

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.y) {
                telemetry.addData("power", 0.4);
                rightMotor.setPower(0.4);
                leftMotor.setPower(0.4);
            } else if (gamepad1.b) {
                telemetry.addData("power", 0.5);
                rightMotor.setPower(0.5);
                leftMotor.setPower(0.5);
            } else if (gamepad1.a) {
                telemetry.addData("power", 0.6);
                rightMotor.setPower(0.6);
                leftMotor.setPower(0.6);
            } else if (gamepad1.x) {
                telemetry.addData("power", 1);
                rightMotor.setPower(1);
                leftMotor.setPower(1);
            } else {
                telemetry.addData("power", 0);
                rightMotor.setPower(0);
                leftMotor.setPower(0);
            }
            telemetry.update();
            idle();
        }

        rightMotor.setPower(0);
        leftMotor.setPower(0);

    }
}
