package org.firstinspires.ftc.teamcode.testing.holonomic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.configuration.HolonomicRobot;

/**
 * Created by thomaslauer on 10/9/16.
 */
@Autonomous(name="HolonomicTest")
public class HolonomicMovementTest extends LinearOpMode {

    HolonomicRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HolonomicRobot();
        robot.initialize(hardwareMap, this);

        telemetry.addData("STATUS", "READY");
        telemetry.update();
        waitForStart();

        //robot.drive.smartDriveAuto(0, 0.5, 2000);
        robot.drive.driveAuto(0, 1, 2000);
        robot.drive.driveAuto(1, 0, 2000);
        //robot.drive.rotateCounterClockwiseAuto(Math.PI/4, 0.5);

        telemetry.addData("STATUS", "finished");
        telemetry.update();
        //sleep(1000);
    }
}
