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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous(name = "sample auto", group = "Linear OpMode")

public class SampleAuto extends LinearOpMode {
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;

    private double currentPosition = 0.8;
    private double currentPosition1 = 0.3; // Start the servo at the middle position
    private int blockNum = 1;
    private int numSamp = 1;
    private double tolerance = 0.1;
    private int bucketX = 10;
    private int bucketY = 60;
    private int pickupX = 20; // way to high right now to not break the claw
    private int pickupY = 44;
    private int pickupOffset = 10;
    private static final double CHANGE_AMOUNT = 0.005;
    public boolean useLiftEncoder = false;
    private int lift_top = 750;
    private int lift_bottom = 0;
    boolean moveStuff = true;
    boolean liftPosition = true;
    boolean done = false;


    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo slide_left = null;
    private Servo slide_right = null;
    private Servo bar_left = null;
    private Servo bar_right = null;
    private Servo left_right_hinge = null;
    private Servo up_down_hinge = null;
    private Servo claw = null;
    //    private CRServo claw = null;
    private Servo top_arm = null;
    private Servo outtake_claw = null;
    private DcMotor lift_left = null;
    private DcMotor lift_right = null;

    // Add variables for slow mode
    private boolean slowMode = false;
    private final double SLOW_MODE_FACTOR = 0.25; // Adjust this value to change the slow mode speed

    //    this function goes straight from the wall to place the specimen
    public void off(){
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    public void forward(){
        leftFrontDrive.setPower(.4);
        leftBackDrive.setPower(.4);
        rightBackDrive.setPower(.4);
        rightFrontDrive.setPower(.4);
    }

    public void reverse(){
        leftFrontDrive.setPower(-.4);
        leftBackDrive.setPower(-.4);
        rightBackDrive.setPower(-.4);
        rightFrontDrive.setPower(-.4);
    }

    public void strafeRight(){
        leftFrontDrive.setPower(0.4);
        leftBackDrive.setPower(-0.4);
        rightBackDrive.setPower(0.4);
        rightFrontDrive.setPower(-0.4);
    }

    public void strafeLeft(){
        leftFrontDrive.setPower(-0.4);
        leftBackDrive.setPower(0.4);
        rightBackDrive.setPower(-0.4);
        rightFrontDrive.setPower(0.4);
    }


    // pick up sample and pass thru
    public void passThru(){
        // position intake arm
        bar_left.setPosition(0.38);
        bar_right.setPosition(0.77);
        left_right_hinge.setPosition(0.72);
        up_down_hinge.setPosition(1.0);
        sleep(100);

        // close claw
        claw.setPosition(0.91);
        sleep(100);
        // open outtake claw and move to correct position
        outtake_claw.setPosition(.2);

        // move intake claw to correct position
        bar_left.setPosition(0.65);
        bar_right.setPosition(0.51);
        left_right_hinge.setPosition(0.72);
        up_down_hinge.setPosition(0.0);

        // position top arm
        top_arm.setPosition(0.7);

        // make sure slides are in
        slide_left.setPosition(0.61);
        slide_right.setPosition(0.49);

        // hand off
        sleep(1500);
        outtake_claw.setPosition(.45);
        sleep(500);
        claw.setPosition(0.55);
        top_arm.setPosition(0.1);
        top_arm.setPosition(0.3);
    }

    // place the sample
    public void place(){
        odo.update();

        double newTime = getRuntime();
        double loopTime = newTime - oldTime;
        double frequency = 1 / loopTime;
        oldTime = newTime;

        // Get the current Position (x & y in mm, and heading in degrees) of the robot
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

        // Get the current Velocity (x & y in mm/sec and heading in degrees/sec)
        Pose2D vel = odo.getVelocity();
        String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);

        //    align with bucket front and back
        while (pos.getX(DistanceUnit.INCH) < bucketX - tolerance || pos.getX(DistanceUnit.INCH) > bucketX + tolerance && opModeIsActive()) {
            if (pos.getX(DistanceUnit.INCH) < bucketX - tolerance){
                forward();
            }
            if (pos.getX(DistanceUnit.INCH) > bucketX + tolerance){
                reverse();
            }
            // Update the position
            pos = odo.getPosition();
            odo.update();
        }
        off();

        //    align with bucket left and right
        while (pos.getY(DistanceUnit.INCH) < bucketY - tolerance && pos.getY(DistanceUnit.INCH) > bucketY + tolerance && opModeIsActive()) {
            if (pos.getY(DistanceUnit.INCH) < bucketY - tolerance){
                strafeLeft();
            }
            if (pos.getY(DistanceUnit.INCH) > bucketY + tolerance){
                strafeRight();
            }
            // Update the position
            pos = odo.getPosition();
            odo.update();
        }
        off();

        //  raise scissor lift
        while (lift_left.getCurrentPosition() + 15 < lift_top && opModeIsActive()) {
            lift_left.setPower(.8);
            lift_right.setPower(lift_left.getPower());
        }
        lift_left.setPower(0);
        lift_right.setPower(lift_left.getPower());

        sleep(100);
        outtake_claw.setPosition(0.2);
        sleep(100);
        top_arm.setPosition(0.8);

        //  back away from bucket
        while (pos.getX(DistanceUnit.INCH) < bucketX + 5 && opModeIsActive()) {
            forward();
            // Update the position
            pos = odo.getPosition();
            odo.update();
        }
        off();
    }

    // grab the next sample based on how many have been placed
    public void grab(){
        odo.update();

        double newTime = getRuntime();
        double loopTime = newTime - oldTime;
        double frequency = 1 / loopTime;
        oldTime = newTime;

        // Get the current Position (x & y in mm, and heading in degrees) of the robot
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

        // Get the current Velocity (x & y in mm/sec and heading in degrees/sec)
        Pose2D vel = odo.getVelocity();
        String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);

        while (pos.getX(DistanceUnit.INCH) < pickupX || lift_left.getCurrentPosition() > lift_bottom && opModeIsActive()) {
            if (pos.getX(DistanceUnit.INCH) < bucketX + 5){
                forward();
            } else {
                off();
            }

            if (lift_left.getCurrentPosition() > lift_bottom){
                lift_left.setPower(-.7);
                lift_right.setPower(lift_left.getPower());
            } else {
                lift_left.setPower(0);
                lift_right.setPower(lift_left.getPower());
            }
            // Update the position
            pos = odo.getPosition();
            odo.update();
        }
        off();

        while (pos.getY(DistanceUnit.INCH) < pickupY + pickupOffset*blockNum- tolerance || pos.getX(DistanceUnit.INCH) > pickupY + pickupOffset*blockNum + tolerance && opModeIsActive()){
            if (pos.getY(DistanceUnit.INCH) < pickupY - tolerance){
                strafeLeft();
            }
            if (pos.getY(DistanceUnit.INCH) > pickupY + tolerance){
                strafeRight();
            }
        }
        off();

        passThru();

    }



    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        lift_left = hardwareMap.get(DcMotor.class, "scl"); //left lift
        lift_right = hardwareMap.get(DcMotor.class, "scr"); // right lift
        slide_left = hardwareMap.get(Servo.class, "sll");
        slide_right = hardwareMap.get(Servo.class, "slr");
        bar_left = hardwareMap.get(Servo.class, "brl");
        bar_right = hardwareMap.get(Servo.class, "brr");
        left_right_hinge = hardwareMap.get(Servo.class, "hlr");
        up_down_hinge = hardwareMap.get(Servo.class, "hud");
        claw = hardwareMap.get(Servo.class, "clw");
//        claw = hardwareMap.get(CRServo.class, "clw");
        top_arm = hardwareMap.get(Servo.class, "tam");
        outtake_claw = hardwareMap.get(Servo.class, "ocw");


        lift_left.setDirection(DcMotor.Direction.REVERSE);
        lift_right.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_left.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        lift_right.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        lift_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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


        // Send telemetry message to indicate successful Encoder reset

        // Wait for the game to start (driver presses PLAY)

//        bar_left.setPosition(0.61);
//        bar_right.setPosition(0.55);
//        left_right_hinge.setPosition(0.72);
//        up_down_hinge.setPosition(0.3);
//        top_arm.setPosition(0.1);
        liftPosition = true;
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

            // Run until the end of the match (driver presses STOP)

            if (moveStuff) {
                top_arm.setPosition(0.8);
                outtake_claw.setPosition(.45);
                moveStuff = false;
            }


            // Request an update from the Pinpoint odometry computer
            odo.update();

            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            oldTime = newTime;

            // Get the current Position (x & y in mm, and heading in degrees) of the robot
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            // Get the current Velocity (x & y in mm/sec and heading in degrees/sec)
            Pose2D vel = odo.getVelocity();
            String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);

            // Set servo positions
            slide_left.setPosition(0.8);
            slide_right.setPosition(0.3);

            if(!done) {
                // place preloaded specimen
                place();
                grab();
                place();
                numSamp += 1;
                grab();
                place();
                numSamp += 1;
                grab();
                place();
                numSamp += 1;
                done = true;
            }


            // Update telemetry
            telemetry.addData("heading: ", pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("position: ", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Lift Left Power", lift_left.getPower());
            telemetry.addData("Lift Right Power", lift_right.getPower());
            telemetry.addData("Lift Left Position", lift_left.getCurrentPosition());
            telemetry.addData("Lift Right Position", lift_right.getCurrentPosition());
            telemetry.update();
            resetRuntime();
        }



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

//                telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
        telemetry.update();
    }
}
