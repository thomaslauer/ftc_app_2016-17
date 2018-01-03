package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.configuration.GearliHardwareTest;

/**
 * Created by fello on 2/27/2017.
 */
@Autonomous(name = "Autonomous Test")
public class AutonomousTest extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        GearliHardwareTest robot =  new GearliHardwareTest();
        robot.initialize(hardwareMap, this);

        waitForStart();

        robot.drive(1);
        sleep(1000);

        robot.drive(2);
    }
}
