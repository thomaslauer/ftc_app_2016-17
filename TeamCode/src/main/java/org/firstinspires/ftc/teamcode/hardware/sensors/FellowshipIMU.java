package org.firstinspires.ftc.teamcode.hardware.sensors;

import android.util.Log;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Created by fello on 1/5/2017.
 */

public class FellowshipIMU {

    public BNO055IMU imu;

    public double currentAngle;
    public double lastAngle;

    public double firstAngle;

    public double piMultiplier = 0;

    public static final double THRESH = 3;

    private static final boolean LOGGING = false;

    public void initialize(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, "IMU");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.temperatureUnit = BNO055IMU.TempUnit.CELSIUS;
        imu.initialize(parameters);

        update();
        resetStartPosition();
    }

    public double getAngularOrientation() {
        return imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.XYZ).thirdAngle;
    }

    public void update() {
        lastAngle = currentAngle;
        currentAngle = getAngularOrientation() + 2 * Math.PI * piMultiplier;

        if(Math.abs(currentAngle - lastAngle) > THRESH && currentAngle < lastAngle) {
            if(LOGGING) Log.d("IMU", "************* OVERFLOW *************");
            piMultiplier += 1;
            currentAngle += 2*Math.PI;
        }

        if(Math.abs(currentAngle - lastAngle) > THRESH && currentAngle > lastAngle) {
            if(LOGGING) Log.d("IMU", "************* OVERFLOW *************");
            piMultiplier -= 1;
            currentAngle -= 2*Math.PI;
        }


        if(LOGGING) Log.d("IMU", "ANGLE IS " + currentAngle);
    }

    public void resetStartPosition() {
        firstAngle = currentAngle;
    }

    public double getCurrentAngle() {
        return currentAngle;
    }

    public double getDeltaAngle() {
        return currentAngle - firstAngle;
    }
}
