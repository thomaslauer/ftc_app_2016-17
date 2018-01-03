package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by fello on 11/6/2016.
 */

@TeleOp(name="ExGripper Julia Oliver")
public class ExampleGripperJuliaOliver extends OpMode {

    final double LEFT_OPEN_POSITION = 0.0;
    final double LEFT_CLOSED_POSITION = 1.0;
    final double RIGHT_OPEN_POSITION = 1.0;
    final double RIGHT_CLOSED_POSITION = 0.0;

    Servo leftGripper;
    Servo rightGripper;


    @Override
    public void init() {
        leftGripper = hardwareMap.servo.get("leftHand");
        rightGripper = hardwareMap.servo.get("rightHand");
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
        leftGripper.setPosition(LEFT_OPEN_POSITION);
        rightGripper.setPosition(RIGHT_OPEN_POSITION);
        }
        if (gamepad1.a) {
        leftGripper.setPosition(LEFT_CLOSED_POSITION);
        rightGripper.setPosition(RIGHT_CLOSED_POSITION);
        }
    }
}
