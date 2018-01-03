package org.firstinspires.ftc.teamcode.hardware.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by Thomas on 2/26/2017.
 */

public class FellowshipUltrasonicSensor extends FellowshipSensor {

    public UltrasonicSensor mUltrasonicSensor;

    @Override
    public void initialize(LinearOpMode opMode) {
        super.initialize(opMode);
        mUltrasonicSensor = mHardwareMap.ultrasonicSensor.get("ultrasonicSensor");
    }

    public double getReadingAverage() {
        while(Double.isNaN(mUltrasonicSensor.getUltrasonicLevel())) {
            mOpMode.idle();
        }

        double sum = 0;
        double numberOfValidMeasurements = 0;

        double minimumMeasurements = 10;

        while(numberOfValidMeasurements < minimumMeasurements) {
            double tempValue = mUltrasonicSensor.getUltrasonicLevel();
            if(!Double.isNaN(tempValue)) {
                sum += tempValue;
                numberOfValidMeasurements++;
            }
            mOpMode.sleep(25);
        }

        return sum / numberOfValidMeasurements;
    }
}
