package org.firstinspires.ftc.teamcode.testing.thrower;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by fello on 9/18/2016.
 */
@Autonomous(name="thrower test")
public class ThrowerTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor rightMotor = hardwareMap.dcMotor.get("rightMotor");
        DcMotor leftMotor = hardwareMap.dcMotor.get("leftMotor");

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double power = 1;

        waitForStart();
        for(double i = 0.0; i < power; i += 0.01){
            rightMotor.setPower(i);
            leftMotor.setPower(i);
            sleep(25);
        }
        while(opModeIsActive()) {
            rightMotor.setPower(power);
            leftMotor.setPower(power);
        }
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }
}
