package org.firstinspires.ftc.teamcode.hardware.configuration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.hardware.components.Thrower;
import org.firstinspires.ftc.teamcode.hardware.sensors.FellowshipUltrasonicSensor;

/**
 * Created by fello on 12/18/2016.
 */

public class LegolessRobot extends HolonomicRobot {
    public Thrower thrower;
    public ColorSensor colorSensor;
    public OpticalDistanceSensor distanceSensor;

    public FellowshipUltrasonicSensor ultrasonicSensor;

    public DcMotor liftMotor;
    public DcMotor doorMotor;

    @Override
    public void initialize(HardwareMap hardwareMap, LinearOpMode opMode) {
        super.initialize(hardwareMap, opMode);
        thrower = new Thrower();
        thrower.initialize(this);

        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        distanceSensor = hardwareMap.opticalDistanceSensor.get("distanceSensor");

        ultrasonicSensor = new FellowshipUltrasonicSensor();
        ultrasonicSensor.initialize(opMode);

        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        doorMotor = hardwareMap.dcMotor.get("doorMotor");
    }
}
