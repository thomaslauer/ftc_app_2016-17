package org.firstinspires.ftc.teamcode.hardware.configuration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by fello on 2/28/2017.
 */

public class GearliHardwareTest implements FellowshipRobot {

    public DcMotor rightMotor;
    public DcMotor leftMotor;

    public UltrasonicSensor ultrasonicSensor;

    public LinearOpMode opMode;

    @Override
    public void initialize(HardwareMap hardwareMap, LinearOpMode opMode) {

        this.opMode = opMode;

        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");

        ultrasonicSensor = hardwareMap.ultrasonicSensor.get("ultrasonic");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive (double distance){
        distance =  distance * 4.1772973503558 * 1120;

        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotor.setMaxSpeed(2000);
        leftMotor.setMaxSpeed(2000);

        rightMotor.setTargetPosition((int)distance);
        leftMotor.setTargetPosition((int)distance);

        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightMotor.setPower(0.75);
        leftMotor.setPower(0.75);

        while (rightMotor.isBusy() && leftMotor.isBusy() && opMode.opModeIsActive()){}

        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }
}
