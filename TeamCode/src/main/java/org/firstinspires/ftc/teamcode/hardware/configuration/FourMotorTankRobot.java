package org.firstinspires.ftc.teamcode.hardware.configuration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by thomaslauer on 9/27/16.
 *
 * Represents a basic four motor tank drive robot.
 */

public class FourMotorTankRobot implements FellowshipRobot {
    OpMode opMode;
    public DcMotor rightMotor1;
    public DcMotor rightMotor2;

    public DcMotor leftMotor1;
    public DcMotor leftMotor2;

    @Override
    public void initialize(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;
        rightMotor1 = hardwareMap.dcMotor.get("rightMotor1");
        rightMotor2 = hardwareMap.dcMotor.get("rightMotor2");
        leftMotor1 = hardwareMap.dcMotor.get("leftMotor1");
        leftMotor2 = hardwareMap.dcMotor.get("leftMotor2");

        rightMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void moveForward(double rotations, double power) {
        double counts = rotations * 1120.0;

        rightMotor1.setTargetPosition((int) (rightMotor1.getCurrentPosition() + counts));
        rightMotor2.setTargetPosition((int) (rightMotor2.getCurrentPosition() + counts));
        leftMotor1.setTargetPosition((int) (leftMotor1.getCurrentPosition() + counts));
        leftMotor2.setTargetPosition((int) (leftMotor2.getCurrentPosition() + counts));

        rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightMotor1.setPower(power);
        rightMotor1.setPower(power);
        leftMotor1.setPower(power);
        leftMotor2.setPower(power);

        while(rightMotor1.isBusy()
                && rightMotor2.isBusy()
                && leftMotor1.isBusy()
                && leftMotor2.isBusy());
    }
}
