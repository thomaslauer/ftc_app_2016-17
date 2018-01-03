package org.firstinspires.ftc.teamcode.testing.holonomic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.configuration.HolonomicRobot;

/**
 * Created by thomaslauer on 11/15/16.
 */
@Autonomous(name = "Auto Turning Test")
public class AutoTurningTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HolonomicRobot robot = new HolonomicRobot();
        robot.initialize(hardwareMap, this);

        waitForStart();
        robot.drive.driveAuto(0.75, 0.75, 1500);
        robot.drive.rotateCounterClockwiseAuto(Math.PI/2, 0.75);
        robot.drive.driveAuto(0.75, 0.75, 1500);
        robot.drive.rotateCounterClockwiseAuto(Math.PI/2, 0.75);
        robot.drive.driveAuto(0.75, 0.75, 1500);
        robot.drive.rotateCounterClockwiseAuto(Math.PI/2, 0.75);
        robot.drive.driveAuto(0.75, 0.75, 1500);
        robot.drive.rotateCounterClockwiseAuto(Math.PI/2, 0.75);
    }
}
