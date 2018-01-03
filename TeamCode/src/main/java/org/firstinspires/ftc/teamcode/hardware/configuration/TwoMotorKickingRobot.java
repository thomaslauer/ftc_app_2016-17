package org.firstinspires.ftc.teamcode.hardware.configuration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by fello on 11/29/2016.
 */

public class TwoMotorKickingRobot extends TwoMotorTankRobot {

    public DcMotor kickMotorRight;
    public DcMotor kickMotorLeft;

    @Override
    public void initialize(HardwareMap hardwareMap, LinearOpMode opMode) {
        super.initialize(hardwareMap, opMode);
        kickMotorRight = hardwareMap.dcMotor.get("kickMotorRight");
        kickMotorLeft = hardwareMap.dcMotor.get("kickMotorLeft");

        kickMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
