package org.firstinspires.ftc.teamcode.hardware.components;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.configuration.LegolessRobot;

/**
 * Created by fello on 12/18/2016.
 */

public class Thrower extends FellowshipHardwareComponent {

    public DcMotor rightMotor;
    public DcMotor leftMotor;

    public double currentSpeed = 0;
    public double targetSpeed = 0;


    public final double rampUpSpeed = 0.005;
    public final double rampDownSpeed = 0.0025;

    public void initialize(LegolessRobot robot) {
        super.initialize(opMode);

        rightMotor = robot.opMode.hardwareMap.dcMotor.get("rightThrowerMotor");
        leftMotor = robot.opMode.hardwareMap.dcMotor.get("leftThrowerMotor");

        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setSpeed(double newTargetSpeed) {
        targetSpeed = newTargetSpeed;
    }

    public void update() {


        Log.d("Motor power", "" + rightMotor.getPower());

        if(currentSpeed < targetSpeed) {
            currentSpeed += rampUpSpeed;
        } else if (currentSpeed > targetSpeed) {
            currentSpeed -= rampDownSpeed;
        }

        rightMotor.setPower(currentSpeed);
        leftMotor.setPower(currentSpeed);

        Log.d("Encoder", "Right Encoder " + rightMotor.getCurrentPosition() + " Left Encoder " + leftMotor.getCurrentPosition());
    }
}
