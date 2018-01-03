package org.firstinspires.ftc.teamcode.hardware.configuration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.components.HolonomicDriveMatrix;
import org.firstinspires.ftc.teamcode.hardware.sensors.FellowshipIMU;

/**
 * Created by fello on 9/18/2016.
 */
public class HolonomicRobot implements FellowshipRobot {

    public DcMotor ne;
    public DcMotor se;
    public DcMotor sw;
    public DcMotor nw;

    public LinearOpMode opMode;

    public HolonomicDriveMatrix drive;

    public FellowshipIMU imu;

    @Override
    public void initialize(HardwareMap hardwareMap, LinearOpMode opMode){
        ne = hardwareMap.dcMotor.get("neMotor");
        se = hardwareMap.dcMotor.get("seMotor");
        sw = hardwareMap.dcMotor.get("swMotor");
        nw = hardwareMap.dcMotor.get("nwMotor");

        imu = new FellowshipIMU();
        imu.initialize(hardwareMap);

        this.opMode = opMode;


        drive = new HolonomicDriveMatrix(this);
    }
}
