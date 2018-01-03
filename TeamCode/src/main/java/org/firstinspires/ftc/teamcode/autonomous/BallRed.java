package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.configuration.LegolessRobot;

/**
 * Created by fello on 1/7/2017.
 */
@Autonomous(name = "BallRed")
public class BallRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LegolessRobot robot = new LegolessRobot();
        robot.initialize(hardwareMap, this);


        telemetry.addData("Status", "READY");
        telemetry.update();
        waitForStart();

        robot.drive.driveAuto(0, 1.1, 1500);
        robot.drive.rotateCounterClockwiseAuto(Math.PI/4, 1);
    }
}
