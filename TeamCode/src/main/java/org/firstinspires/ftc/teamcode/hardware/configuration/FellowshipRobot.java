package org.firstinspires.ftc.teamcode.hardware.configuration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Thomas on 9/11/2016.
 * <p>
 * Represents all the hardware on the robot, removes the declarations from the individual
 * OpModes. Use {@link #initialize(HardwareMap, LinearOpMode)} to populate the motor and sensor variables
 *
 * To use, implement FellowshipRobot and fill with respective variables.
 * </p>
 */
public interface FellowshipRobot {
    void initialize(HardwareMap hardwareMap, LinearOpMode opMode);
}
