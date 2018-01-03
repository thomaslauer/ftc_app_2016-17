package org.firstinspires.ftc.teamcode.hardware.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by thomaslauer on 12/10/16.
 */
public class FellowshipSensor {
    public LinearOpMode mOpMode;
    public HardwareMap mHardwareMap;

    public void initialize(LinearOpMode opMode) {
        this.mOpMode = opMode;
        this.mHardwareMap = opMode.hardwareMap;
    }
}
