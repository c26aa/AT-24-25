package org.firstinspires.ftc.teamcode;

/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/*
This opmode shows how to use the goBILDA® Pinpoint Odometry Computer.
The goBILDA Odometry Computer is a device designed to solve the Pose Exponential calculation
commonly associated with Dead Wheel Odometry systems. It reads two encoders, and an integrated
system of senors to determine the robot's current heading, X position, and Y position.

it uses an ESP32-S3 as a main cpu, with an STM LSM6DSV16X IMU.
It is validated with goBILDA "Dead Wheel" Odometry pods, but should be compatible with any
quadrature rotary encoder. The ESP32 PCNT peripheral is speced to decode quadrature encoder signals
at a maximum of 40mhz per channel. Though the maximum in-application tested number is 130khz.

The device expects two perpendicularly mounted Dead Wheel pods. The encoder pulses are translated
into mm and their readings are transformed by an "offset", this offset describes how far away
the pods are from the "tracking point", usually the center of rotation of the robot.

Dead Wheel pods should both increase in count when moved forwards and to the left.
The gyro will report an increase in heading when rotated counterclockwise.

The Pose Exponential algorithm used is described on pg 181 of this book:
https://github.com/calcmogul/controls-engineering-in-frc

For support, contact tech@gobilda.com

-Ethan Doak
 */

@Autonomous(name="auto test", group="Linear OpMode")
//@Disabled

public class PinpointAuto extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf = null;
    private DcMotor lb = null;
    private DcMotor rf = null;
    private DcMotor rb = null;

    private CRServo intake1 = null; //port 5 control hub.
    private CRServo intake2 = null; //port 0 exp hub.
    private Servo wrist1 = null; //port 1 expansion hub. right
    private Servo wrist2 = null; //port 4 control hub. left
    private Servo shoulder1 = null; //port 2 control hub. right
    private Servo shoulder2 = null; //port 3 control hub. left
    static final double WRIST1_DOWN = .75;
    static final double WRIST1_UP = 0.2;

    //TODO: FIND POSITIONS wrist 2
    static final double WRIST2_DOWN = 0.675;
    static final double WRIST2_UP = 0.2;

    static final double INTAKE_PWR = 0.95;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-80.0, 162.5); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
//        odo.recalibrateIMU();
        odo.resetPosAndIMU();

        double startTime = System.currentTimeMillis();

        // Initialize the drive system variables.
        lf = hardwareMap.get(DcMotor.class, "leftFront");
        lb = hardwareMap.get(DcMotor.class, "leftBack");
        rf = hardwareMap.get(DcMotor.class, "rightFront");
        rb = hardwareMap.get(DcMotor.class, "rightBack");

        intake1 = hardwareMap.get(CRServo.class, "intake1");
        intake2 = hardwareMap.get(CRServo.class, "intake2");
        wrist1 = hardwareMap.get(Servo.class, "wrist1");
        wrist2 = hardwareMap.get(Servo.class, "wrist2");
        shoulder1 = hardwareMap.get(Servo.class, "shoulder1");
        shoulder2 = hardwareMap.get(Servo.class, "shoulder2");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);


        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wrist1.setPosition(WRIST1_UP);
        wrist2.setPosition(WRIST2_UP);

        intake2.setPower(0);
        intake1.setPower(-0);

        // Send telemetry message to indicate successful Encoder reset

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Status", "Initialized");
            telemetry.addData("X offset", odo.getXOffset());
            telemetry.addData("Y offset", odo.getYOffset());
            telemetry.addData("Device Version Number:", odo.getDeviceVersion());
            telemetry.addData("Device Scalar", odo.getYawScalar());
            telemetry.update();

            // Wait for the game to start (driver presses START)
            waitForStart();
            resetRuntime();


            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

            /*
            Request an update from the Pinpoint odometry computer. This checks almost all outputs
            from the device in a single I2C read.
             */
                odo.update();

                double newTime = getRuntime();
                double loopTime = newTime - oldTime;
                double frequency = 1 / loopTime;
                oldTime = newTime;


            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
                Pose2D pos = odo.getPosition();
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
                Pose2D vel = odo.getVelocity();
                String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Velocity", velocity);

                intake2.setPower(-INTAKE_PWR*.5);
                intake1.setPower(INTAKE_PWR*.5);
                while (pos.getX(DistanceUnit.INCH) < 50) {

                    lf.setPower(.4);
                    lb.setPower(.4);
                    rb.setPower(.4);
                    rf.setPower(.4);
                    odo.update();
                    pos = odo.getPosition();

                    telemetry.addData("position: ", pos.getX(DistanceUnit.INCH));
                    telemetry.update();

                }

                //STRAFELEFT THEN FORWARD THEN RIGHT THEN BACK
                lf.setPower(0);
                lb.setPower(0);
                rb.setPower(0);
                rf.setPower(0);
//                forwardDrive(.3, 155, pos.getX(DistanceUnit.INCH));
                if(pos.getX(DistanceUnit.INCH) >=38&& runtime.seconds()<7.0) {
                    pos = odo.getPosition();
                    telemetry.addData("test: ", pos.getX(DistanceUnit.INCH));
                    telemetry.addData("secs: ", runtime.seconds());

                    telemetry.update();
                    wrist1.setPosition(WRIST1_DOWN);
                    intake2.setPower(1);
                    intake1.setPower(-1);
                    sleep(5000);

                }
                wrist1.setPosition(WRIST1_UP);
                intake2.setPower(0);
                intake1.setPower(0);
                //straferight
                while (pos.getX(DistanceUnit.INCH)>36) {
                    lf.setPower(-.3);
                    lb.setPower(-.3);
                    rb.setPower(-.3);
                    rf.setPower(-.3);
                    pos = odo.getPosition();
                    telemetry.addData("backing up, pos: ", pos.getX(DistanceUnit.INCH));
                    telemetry.addData("secs: ", runtime.seconds());
                }
                //strafeleft
                lf.setPower(0);
                lb.setPower(0);
                rb.setPower(0);
                rf.setPower(0);


//                forwardDrive(-.8, 10, pos.getX(DistanceUnit.INCH));
//                strafeLeft(-.8, 40, pos.getY(DistanceUnit.INCH)); //strafe right
//                intake.setPower(-.8);
//                forwardDrive(-.6, 5, pos.getX(DistanceUnit.INCH));
//                wrist1.setPosition(WRIST1_UP);
//                strafeLeft(.8, 40, pos.getY(DistanceUnit.INCH));
//                wrist1.setPosition(WRIST1_DOWN);
//                intake.setPower(.8);
//                wrist1.setPosition(WRIST1_UP);
//                intake.setPower(0);
//                forwardDrive(-.8, 175, pos.getX(DistanceUnit.INCH));




            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            */
                telemetry.addData("Status", odo.getDeviceStatus());

                telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

                telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
                telemetry.update();
            }
        }
    }
    public void forwardDrive(double power, double position, double pos) {

        System.out.println("in forward drive");



        //wait until reaches position
//                while (pos < position && opModeIsActive()) {
//                    lf.setPower(power);
//                    lb.setPower(power);
//                    rb.setPower(power);
//                    rf.setPower(power);
//
//                    odo.update();
//
//                    telemetry.addData("position: ", pos);
//                    telemetry.addData("lf, lb, rb, rf", power);
//                    telemetry.update();
//
//                }
//
//                lf.setPower(0);
//                lb.setPower(0);
//                rb.setPower(0);
//                rf.setPower(0);
    }
}