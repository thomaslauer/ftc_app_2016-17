package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.configuration.LegolessRobot;

/**
 * Created by fello on 3/4/2017.
 */
@Autonomous(name = "Light Sensor Test")
public class LightSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LegolessRobot robot = new LegolessRobot();
        robot.initialize(hardwareMap, this);

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Distance Sensor ", robot.distanceSensor.getLightDetected());
            telemetry.update();
        }
    }
}
