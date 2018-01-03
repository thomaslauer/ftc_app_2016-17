package org.firstinspires.ftc.teamcode.hardware.components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.configuration.HolonomicRobot;

/**
 * Created by FTC7123 on 9/17/2016.
 *
 * Controls the holonomic robot drive train using matrix rotation to calculate powers
 */
public class HolonomicDriveMatrix {
    /**
     * DcMotors, corresponding to position on robot.
     * (ne = northeast, se = southeast, etc.)
     */
    public DcMotor ne, se, sw, nw;

    LinearOpMode opMode;

    HolonomicRobot robot;

    public int speed = 2000;

    public int speedSlow = 1500;

    public int speedFast = 2500;

    public final double encoderTicksToMeters = 0.00028484;

    public HolonomicDriveMatrix(HolonomicRobot robot) {
        this.ne = robot.ne;
        this.se = robot.se;
        this.sw = robot.sw;
        this.nw = robot.nw;
        this.opMode = robot.opMode;
        this.robot = robot;

        resetMotorSpeeds();
    }

    public void drive(double x, double y, double rotation) {
        y = -y;
        resetMotorSpeeds();
        ne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        se.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ne.setPower(calculateMotorPower(x, y, -Math.PI/4) + rotation);
        se.setPower(calculateMotorPower(x, y, -Math.PI/4 + 1*Math.PI/2) + rotation);
        sw.setPower(calculateMotorPower(x, y, -Math.PI/4 + 2*Math.PI/2) + rotation);
        nw.setPower(calculateMotorPower(x, y, -Math.PI/4 + 3*Math.PI/2) + rotation);
    }

    public void smartDrive(double x, double y, double rotation) {
        double x2 = rotateClockwiseX(x, y, robot.imu.getDeltaAngle());
        double y2 = rotateClockwiseY(x, y, robot.imu.getDeltaAngle());
        drive(x2, y2, rotation);
    }

    public void driveAuto(double distanceX, double distanceY, int maxSpeed) {

        distanceY = -distanceY;

        distanceX = distanceX / encoderTicksToMeters;
        distanceY = distanceY / encoderTicksToMeters;

        ne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        se.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        nw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        se.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        nw.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        opMode.idle();

        double r = Math.sqrt((distanceX * distanceX) + (distanceY * distanceY));

        // sets the max speed of the motors, we regulate the power using setPower below
        // I think this works?
        ne.setMaxSpeed(maxSpeed);
        se.setMaxSpeed(maxSpeed);
        sw.setMaxSpeed(maxSpeed);
        nw.setMaxSpeed(maxSpeed);

        // sets the target position for the motors
        ne.setTargetPosition((int) (rotateClockwiseY(distanceX, distanceY, -Math.PI/4) + ne.getCurrentPosition()));
        se.setTargetPosition((int) (rotateClockwiseY(distanceX, distanceY, -Math.PI/4 + 1*Math.PI/2) + se.getCurrentPosition()));
        sw.setTargetPosition((int) (rotateClockwiseY(distanceX, distanceY, -Math.PI/4 + 2*Math.PI/2) + sw.getCurrentPosition()));
        nw.setTargetPosition((int) (rotateClockwiseY(distanceX, distanceY, -Math.PI/4 + 3*Math.PI/2) + nw.getCurrentPosition()));

        // sets the motor power. It divides the power by the radius so the maximum would be 1
        ne.setPower(rotateClockwiseY(distanceX, distanceY, -Math.PI/4) / r);
        se.setPower(rotateClockwiseY(distanceX, distanceY, -Math.PI/4 + 1*Math.PI/2) / r);
        sw.setPower(rotateClockwiseY(distanceX, distanceY, -Math.PI/4 + 2*Math.PI/2) / r);
        nw.setPower(rotateClockwiseY(distanceX, distanceY, -Math.PI/4 + 3*Math.PI/2) / r);

        // wait for motors to finish
        while(areMotorsBusy() && opMode.opModeIsActive()) {
            robot.imu.update();
            robot.opMode.idle();
        }

        ne.setPower(0);
        se.setPower(0);
        sw.setPower(0);
        nw.setPower(0);

        // stop motors
        ne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        se.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        nw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        opMode.sleep(100);
    }

    public void rotate(double speed, long time) {
        ne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        se.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ne.setPower(speed);
        se.setPower(speed);
        sw.setPower(speed);
        nw.setPower(speed);

        opMode.sleep(time);

        ne.setPower(0);
        se.setPower(0);
        sw.setPower(0);
        nw.setPower(0);
    }

    public void rotateAuto(double angle, double speed) {
        if(angle > 0) {
            rotateCounterClockwiseAuto(angle, speed);
        }
        if(angle < 0) {
            rotateClockwiseAuto(-angle, speed);
        }
    }

    public void rotateCounterClockwiseAuto(double angle, double speed) {
        resetMotorSpeeds();
        robot.imu.update();

        double startPosition = robot.imu.getDeltaAngle();

        ne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        se.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ne.setPower(-speed);
        se.setPower(-speed);
        sw.setPower(-speed);
        nw.setPower(-speed);

        while(opMode.opModeIsActive() && robot.imu.getDeltaAngle() < angle + startPosition) {
            robot.imu.update();
            opMode.idle();

            //Log.d("IMU", "DELTA ANGLE: " + (angle - robot.imu.getDeltaAngle()));
        }

        ne.setPower(0);
        se.setPower(0);
        sw.setPower(0);
        nw.setPower(0);
    }

    public void rotateClockwiseAuto(double angle, double speed) {
        resetMotorSpeeds();
        robot.imu.update();

        double startPosition = robot.imu.getDeltaAngle();

        ne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        se.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ne.setPower(speed);
        se.setPower(speed);
        sw.setPower(speed);
        nw.setPower(speed);

        while(opMode.opModeIsActive() && robot.imu.getDeltaAngle() > angle + startPosition) {
            robot.imu.update();
            opMode.idle();

            //Log.d("IMU", "angle: " + robot.imu.getDeltaAngle() + " target: " + angle);
        }

        ne.setPower(0);
        se.setPower(0);
        sw.setPower(0);
        nw.setPower(0);
    }

    /**
     * Calculates the power for a motor, given the rectangular coordinates to move and the
     * angle the motor is mounted at
     * @param x horizontal direction
     * @param y vertical direction
     * @param theta angle the motor is mounted at, in radians counter clockwise from positive x
     * @return the power of the motor
     */
    public double calculateMotorPower(double x, double y, double theta) {
        double rotatedPower = rotateClockwiseY(x, y, theta);
        double r = Math.sqrt(x * x + y * y);
        return rotatedPower * r;
    }
    
    /**
     * Rotates a coordinate clockwise around the origin by an angle, only for the X coordinate
     * @param x X coordinate
     * @param y Y coordinate
     * @param theta The angle to rotate
     * @return The X coordinate of the new point
     */
    public double rotateClockwiseX(double x, double y, double theta) {
        return x * Math.cos(theta) + y * Math.sin(theta);
    }
    
    /**
     * Rotates a coordinate clockwise around the origin by an angle, only for the Y coordinate
     * @param x X coordinate
     * @param y Y coordinate
     * @param theta The angle to rotate
     * @return The Y coordinate of the new point
     */
    public double rotateClockwiseY(double x, double y, double theta) {
        return -x * Math.sin(theta) + y * Math.cos(theta);
    }
    
    /**
     * Rotates a coordinate counter clockwise around the origin by an angle, only for the X coordinate
     * @param x X coordinate
     * @param y Y coordinate
     * @param theta The angle to rotate
     * @return The X coordinate of the new point
     */
    public double rotateCounterClockwiseX(double x, double y, double theta) {
        return x * Math.cos(theta) - y * Math.sin(theta);
    }
    
    /**
     * Rotates a coordinate counter clockwise around the origin by an angle, only for the Y coordinate
     * @param x X coordinate
     * @param y Y coordinate
     * @param theta The angle to rotate
     * @return The X coordinate of the new point
     */
    public double rotateCounterClockwiseY(double x, double y, double theta) {
        return x * Math.sin(theta) + y * Math.cos(theta);
    }

    public boolean areMotorsBusy() {
        int numFinished = 0;

        int thresh = 50;

        if(Math.abs(ne.getCurrentPosition() - ne.getTargetPosition()) < thresh) numFinished++;
        if(Math.abs(se.getCurrentPosition() - se.getTargetPosition()) < thresh) numFinished++;
        if(Math.abs(sw.getCurrentPosition() - sw.getTargetPosition()) < thresh) numFinished++;
        if(Math.abs(nw.getCurrentPosition() - nw.getTargetPosition()) < thresh) numFinished++;

//        Log.d("Holonomic", "ne: " + Math.abs(ne.getCurrentPosition() - ne.getTargetPosition())
//                + "\tse: " + Math.abs(se.getCurrentPosition() - se.getTargetPosition())
//                + "\tsw: " + Math.abs(sw.getCurrentPosition() - sw.getTargetPosition())
//                + "\tnw: " + Math.abs(nw.getCurrentPosition() - nw.getTargetPosition()));

        return numFinished <= 3;
    }

    public void resetMotorSpeeds() {
        ne.setMaxSpeed(3000);
        se.setMaxSpeed(3000);
        sw.setMaxSpeed(3000);
        nw.setMaxSpeed(3000);
    }
}
