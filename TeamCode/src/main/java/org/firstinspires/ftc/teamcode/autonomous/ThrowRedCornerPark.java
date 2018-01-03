package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.configuration.LegolessRobot;

/**
 * Created by fello on 3/31/2017.
 */
@Autonomous(name = "Throw Red Corner Park")
public class ThrowRedCornerPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LegolessRobot robot = new LegolessRobot();
        robot.initialize(hardwareMap, this);
        double liftMotorSpeed = 0.125;
        DcMotor liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double doorMotorSpeed = 0.05;
        DcMotor doorMotor = hardwareMap.dcMotor.get("doorMotor");

        telemetry.addData("Status", "READY");
        telemetry.update();
        waitForStart();

        robot.thrower.setSpeed(0.435);
        while(robot.thrower.currentSpeed < robot.thrower.targetSpeed && opModeIsActive()) {
            robot.thrower.update();
        }

        doorMotor.setPower(-doorMotorSpeed);
        sleep(1500);
        liftMotor.setPower(liftMotorSpeed);
        sleep(3000);
        liftMotor.setPower(liftMotorSpeed * 2);
        sleep(3000);
        robot.doorMotor.setPower(0);

        robot.thrower.setSpeed(0);
        while(robot.thrower.currentSpeed > robot.thrower.targetSpeed && opModeIsActive()) {
            robot.thrower.update();
            liftMotor.setPower(0);
        }

        sleep(8000);

        robot.drive.driveAuto(-1.5, 1, 1500);
        sleep(3000);
    }
}
