package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.configuration.LegolessRobot;

/**
 * Created by fello on 1/10/2017.
 */
@Autonomous(name="Color Test")
public class ColorSensorTest extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {

        LegolessRobot robot = new LegolessRobot();
        robot.initialize(hardwareMap, this);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("red", robot.colorSensor.red());
            telemetry.addData("green", robot.colorSensor.green());
            telemetry.addData("blue", robot.colorSensor.blue());
            telemetry.update();
        }

//        if (robot.colorSensor.red() > robot.colorSensor.blue()){
//            robot.drive.driveAuto(-0.125 , 0 , 1000);
//        }else{
//            robot.drive.driveAuto(0.225 , 0 , 1000);
//        }
//
//        robot.drive.driveAuto(0 , -0.15 , 1000);
//        sleep(200);
//        robot.drive.driveAuto(0 , 0.15 , 1000);
    }
}
