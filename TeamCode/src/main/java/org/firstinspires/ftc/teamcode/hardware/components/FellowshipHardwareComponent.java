package org.firstinspires.ftc.teamcode.hardware.components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by fello on 12/18/2016.
 */

public abstract class FellowshipHardwareComponent {

    public LinearOpMode opMode;

    public void initialize(LinearOpMode opMode) {
        this.opMode = opMode;
    }
}
