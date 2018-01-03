package org.firstinspires.ftc.teamcode.hardware.configuration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by thomaslauer on 9/27/16.
 *
 * Represents a basic two motor tank drive robot.
 */

public class TwoMotorTankRobot implements FellowshipRobot {

    public DcMotor rightMotor;
    public DcMotor leftMotor;

    public LinearOpMode opMode;

    @Override
    public void initialize(HardwareMap hardwareMap, LinearOpMode opMode) {

        this.opMode = opMode;

        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
