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


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
import static org.firstinspires.ftc.teamcode.Constants.*;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

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

@Autonomous(name = "rotate specimen auto", group = "Linear OpMode")
//@Disabled

public class PinpointAuto2 extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;

    private double currentPosition = 0.8;
    private double currentPosition1 = 0.3; // Start the servo at the middle position
    private int blockNum = 0;
    private int numSamp = 1;
    private int sampX = 38;
    private int sampY = -48;
    private int laneX = 22;
    private int laneY = -33;
    //    private  int dropX = 18;
    private  int dropX = 21;

    private double pickupX = 0.5; // way to high right now to not break the claw
    private int pickupY = -49;
    private double placeX = 29.5;
    private int placeY = 20;
    private int placementOffset = -3;
    private int sampOffset = -8;
    public boolean useLiftEncoder = false;
    public int lift_target = 0;
    boolean moveStuff = true;
    boolean liftPosition = true;
    boolean done = false;
    private double heading = 0.0;
    //    private int distOff = 9;
    private int distOff = 2;
    private double headingCorrectSpeed = 0.2;
    private double maxSpeed = 0.75;
    private double speed = maxSpeed;
    private double maxSpeed2 = 0.95;
    private double speed2 = maxSpeed2; // max value is 0.89
    private double targetX = 0;
    private double targetY = 0;
    private double placeHeading = -120;
    private double pickHeading = -30;
    private double moveHeading = -90;
    private double tolerance = 0.1;
    private double logAdd = 1.01;
    private double axial = 0.0;
    private double lateral = 0.0;
    private double yaw = 0.0;
    private double bigYawSpeed = 0.4;
    private double correctionSpeed = 0.1;
    private Limelight3A limelight;
    private Servo leftRightHinge = null;
    private Servo barl = null;
    private Servo barr = null;
    final double max_pos = HINGE_LEFT;
    final double mid_pos = HINGE_MIDDLE;
    final double min_pos = HINGE_RIGHT;
    private double bld = 0.38;
    private double brd = 0.77;
    private double blm = bld + 0.17;
    private double brm = brd - 0.17;
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

    public void off(){
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        speed = maxSpeed;
        speed2 = maxSpeed2;
        headingCorrect();
    }

    public void move(){
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

        // Get the current Velocity (x & y in mm/sec and heading in degrees/sec)
        Pose2D vel = odo.getVelocity();
        String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);

        pos = odo.getPosition();
        odo.update();
        heading = pos.getHeading(AngleUnit.DEGREES);

        yaw = 0;
        if (heading > 0.1){
            yaw = 0.05;
        }
        if (heading < -0.1){
            yaw = -0.05;
        }

        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > speed2+0.1) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void rotate(){
        double leftFrontPower = yaw;
        double rightFrontPower = -yaw;
        double leftBackPower = yaw;
        double rightBackPower = -yaw;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void headingCorrect(){
        odo.update();

        double newTime = getRuntime();
        double loopTime = newTime - oldTime;
        double frequency = 1 / loopTime;
        oldTime = newTime;

        // Get the current Position (x & y in mm, and heading in degrees) of the robot
        Pose2D pos = odo.getPosition();
        odo.update();

        heading = pos.getHeading(AngleUnit.DEGREES);
        telemetry.addData("heading", heading);
        telemetry.addData("x", pos.getX(DistanceUnit.INCH));
        telemetry.addData("y", pos.getY(DistanceUnit.INCH));
        telemetry.update();

        while (heading < -0.05 || heading > 0.05){
            pos = odo.getPosition();
            odo.update();
            heading = pos.getHeading(AngleUnit.DEGREES);

            telemetry.addData("heading", heading);
            telemetry.addData("x", pos.getX(DistanceUnit.INCH));
            telemetry.addData("y", pos.getY(DistanceUnit.INCH));
            telemetry.update();

//            if it's rotating too much than reverse heading greater than less than
            if (heading > 0.05){
                leftFrontDrive.setPower(headingCorrectSpeed);
                leftBackDrive.setPower(headingCorrectSpeed);
                rightBackDrive.setPower(-headingCorrectSpeed);
                rightFrontDrive.setPower(-headingCorrectSpeed);
            }
            if (heading < -0.05){
                leftFrontDrive.setPower(-headingCorrectSpeed);
                leftBackDrive.setPower(-headingCorrectSpeed);
                rightBackDrive.setPower(headingCorrectSpeed);
                rightFrontDrive.setPower(headingCorrectSpeed);
            }

        }

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        sleep(10);
    }

    public void pickupClawAction(){
        sleep(300);
        outtake_claw.setPosition(.45);
        sleep(300);
        top_arm.setPosition(0.8);
    }

    public void limelightAction(){
        slide_left.setPosition(LEFT_SLIDES_IN);
        slide_right.setPosition(RIGHT_SLIDES_OUT);
        sleep(100);

        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();

        leftRightHinge.setPosition(mid_pos);
        barl.setPosition(blm);
        barr.setPosition(brm);
        up_down_hinge.setPosition(WRIST_MIDDLE);
        sleep(400);

        result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());

                double degreeToCM = 3.9/12; // CM per degree
                double hingeToCM = -0.1/8; // hinge servo position per CM
                double slideToCM = 0.1/6; // slide servo position per CM


                double cmx = result.getTx()*degreeToCM;

                double arm_pos = mid_pos + (cmx * hingeToCM);
                if (arm_pos < min_pos) {
                    arm_pos = min_pos;
                } else if (arm_pos > max_pos) {
                    arm_pos = max_pos;
                }

                if (arm_pos > 0 && arm_pos < 1){
                    leftRightHinge.setPosition(arm_pos);
                    telemetry.addData("cmx", cmx);
                    telemetry.addData("intended arm pos", mid_pos + (cmx * hingeToCM));
                    telemetry.addData("arm pos position good", arm_pos);
                } else {
                    telemetry.addData("arm pos position wrong", arm_pos);
                }

                // arm down
                sleep(200);
//                        bar_left.setPosition(0.38);
//                        bar_right.setPosition(0.77);
                bar_left.setPosition(0.42);
                bar_right.setPosition(0.73);

                double claw_dist = 14.5; // distance from claw to hinge
                double camera_dist = 6.5; // distance from camera to hinge
                double clawy = Math.sqrt(Math.abs(claw_dist*claw_dist - (cmx*cmx))); // distance from claw to hinge just on y-axis after being rotated
                double samp_dist = camera_dist + result.getTy()*degreeToCM; // distance from sample to hinge on y-axis
                double distY = samp_dist - clawy; // distance from sample to rotated claw/

                telemetry.addData("dist y", distY);

                double slide_posL = slide_left.getPosition();
                double slide_posR = slide_right.getPosition();

                double next_posL = slide_posL - distY*slideToCM;
                double next_posR = slide_posR + distY*slideToCM;

                telemetry.addData("intended slide position", next_posR);

                double new_posL = Math.min(Math.max(next_posL, LEFT_SLIDES_IN), LEFT_SLIDES_OUT);
                double new_posR = Math.min(Math.max(next_posR, RIGHT_SLIDES_IN), RIGHT_SLIDES_OUT);

                if (new_posL > 0 && new_posR > 0 && new_posL < 1 && new_posR < 1){
                    telemetry.addData("slide position", new_posL);
                    telemetry.addData("slide position", new_posR);
                    slide_left.setPosition(new_posL);
                    slide_right.setPosition(new_posR);
                } else {
                    telemetry.addData("slide position wrong", new_posL);
                    telemetry.addData("slide position wrong", new_posR);
                }

                up_down_hinge.setPosition(WRIST_DOWN);
                claw.setPosition(CLAW_OPEN);
                sleep(200);
                bar_left.setPosition(0.383);
                bar_right.setPosition(0.767);
                up_down_hinge.setPosition(WRIST_DOWN);
                sleep(200);
                claw.setPosition(CLAW_CLOSED);
                sleep(200);
                up_down_hinge.setPosition(WRIST_MIDDLE);

                telemetry.update();
            }
        }
    }


    public void placeSpecimen(){
        off();
        liftPosition = true;
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


        // Lift control logic
        if (liftPosition) {
            useLiftEncoder = true;
            lift_target = 490;
            if (blockNum > 0){
                lift_target = 430;
            } if (blockNum == 1){
//                lift_target = 420;
                lift_target = 420;
            }

            liftPosition = false;
        }

        // Drive motor control logic
        while (pos.getX(DistanceUnit.INCH) < placeX+distOff*blockNum && opModeIsActive()) {
            pos = odo.getPosition();
            odo.update();

            double dist = Math.abs(pos.getX(DistanceUnit.INCH) - (placeX+distOff*blockNum));
            speed = Math.min(Math.log(dist/4+logAdd), maxSpeed);

            targetX = 29+distOff*blockNum;
            targetY = placementOffset*(blockNum*2);
            telemetry.addData("lift val", lift_left.getCurrentPosition() + 15);
            heading = pos.getHeading(AngleUnit.DEGREES);
            // Update the lift motors while driving
//            uncomment once lift is ready
            if (lift_target > lift_left.getCurrentPosition() + 15) {
                lift_left.setPower(.95);
            } else if (lift_target < lift_left.getCurrentPosition() - 15) {
                lift_left.setPower(-.8);
            } else {
                lift_left.setPower(0);
            }
            lift_right.setPower(lift_left.getPower());

            if (blockNum > 0 && pos.getY(DistanceUnit.INCH) < -7 + placementOffset*blockNum){
                axial = 0.3;
                lateral = -speed2;
            } else if (blockNum > 0 && pos.getY(DistanceUnit.INCH) > -7 + placementOffset*blockNum+0.1){
                axial = 0.4;
                lateral = 0.05;
            } else {
                axial = speed;
                lateral= 0;
            }

            move();
        } while (pos.getX(DistanceUnit.INCH) > placeX+2+distOff*blockNum && opModeIsActive()){
            axial = -0.2;
            lateral= 0;
            move();
        }

        // Stop the drive motors
        off();

        // Wait for .2 seconds
        sleep(200);
        // Update the lift target after driving
        if (!liftPosition) {
            telemetry.addData("lift target val", lift_left.getCurrentPosition() + 15);
            telemetry.update();
            useLiftEncoder = true;
//            lift_target = 620;
            if (blockNum > 0){
                lift_target = 620;
            } else {
                lift_target = 635;
            }
        }

        //  pull claw back and lift scissor lift to place sample
        top_arm.setPosition(.2);
//        uncomment once lift is ready
        while (lift_target > lift_left.getCurrentPosition() + 15 && opModeIsActive()) {
            telemetry.addData("lift target val", lift_left.getCurrentPosition() + 15);
            telemetry.update();
            if (lift_target > lift_left.getCurrentPosition() + 15) {
                lift_left.setPower(.8);
            } else if (lift_target < lift_left.getCurrentPosition() - 15) {
                lift_left.setPower(-.5);
            } else {
                lift_left.setPower(0);
            }
            lift_right.setPower(lift_left.getPower());
        }
        lift_left.setPower(0);
        lift_right.setPower(lift_left.getPower());

        // Adjust the outtake claw and top arm positions
        sleep(100);
        outtake_claw.setPosition(.2);
        top_arm.setPosition(.1);
        lift_target = 0;

        //  Lower scissor lift
//        uncomment once lift is ready
        while (lift_target < lift_left.getCurrentPosition() && opModeIsActive()) {
            telemetry.addLine("shivatron");
            if (lift_target > lift_left.getCurrentPosition()) {
                lift_left.setPower(.8);
            } else if (lift_target < lift_left.getCurrentPosition()) {
                lift_left.setPower(-.6);
            } else {
                lift_left.setPower(0);
            }
            lift_right.setPower(lift_left.getPower());
        }
        lift_left.setPower(0);
        lift_right.setPower(lift_left.getPower());

        if (lift_target > lift_left.getCurrentPosition() + 15){
            lift_left.setPower(0);
            lift_right.setPower(lift_left.getPower());
        }
    }

    // this function goes from the placed specimen to the first sample on the field
    public void toSamp(){
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

        off();

        //    reverse closer to wall to avoid hitting the corner of the submersible
        if (blockNum == 1){
            while (pos.getX(DistanceUnit.INCH) > laneX && opModeIsActive()) {
                pos = odo.getPosition();
                odo.update();

                double dist = Math.abs(pos.getX(DistanceUnit.INCH) - laneX);
                speed = Math.min(Math.log(dist/4+logAdd), maxSpeed);

                targetX = laneX;
                targetY = 0;

                telemetry.addData("y", pos.getY(DistanceUnit.INCH));
                telemetry.addData("x", pos.getX(DistanceUnit.INCH));
                telemetry.addLine("sub to samp");
                telemetry.update();

                axial = -speed;
                lateral = 0;
                move();
            }
            off();
        }

        //  drive to the lane between the submersible and block
        while (pos.getY(DistanceUnit.INCH) > laneY+sampOffset*numSamp && opModeIsActive()) {
            targetX = laneX;
            targetY = laneY;

            pos = odo.getPosition();
            odo.update();

            double dist = Math.abs(pos.getY(DistanceUnit.INCH) - laneY);
            speed2 = Math.min(Math.log(dist/2+logAdd), maxSpeed2);

            telemetry.addData("y", pos.getY(DistanceUnit.INCH));
            telemetry.addData("x", pos.getX(DistanceUnit.INCH));
            telemetry.update();

            axial = 0;
            lateral = speed2;

            if (blockNum > 1){
                axial = speed;
                lateral = 0;
            } else {
                if (pos.getX(DistanceUnit.INCH) < targetX - 0.1){
                    axial = 0.2;
                } else if (pos.getX(DistanceUnit.INCH) > targetX + 0.2){
                    axial = -0.2;
                }
            }

            move();
        }
    }

    //    push sample straight into human player zone (this function will be run 3 times)
    public void dropOff(){
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

        pos = odo.getPosition();
        odo.update();

        heading = pos.getHeading(AngleUnit.DEGREES);
        while (heading > pickHeading + tolerance || heading < pickHeading - tolerance){
            pos = odo.getPosition();
            odo.update();
            heading = pos.getHeading(AngleUnit.DEGREES);
            telemetry.addData("heading", heading);
            double newYawSpeed = bigYawSpeed;
            if (Math.abs(heading-pickHeading) < 2){
                newYawSpeed =  headingCorrectSpeed;
            }
            telemetry.update();
            if (heading > pickHeading + tolerance){
                yaw = newYawSpeed;
                rotate();
            }
            else if (heading < pickHeading - tolerance){
                yaw = -newYawSpeed;
                rotate();
            }
        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);

        limelightAction();

        odo.update();
        heading = pos.getHeading(AngleUnit.DEGREES);
        while (heading > placeHeading + tolerance || heading < placeHeading - tolerance){
            pos = odo.getPosition();
            odo.update();
            heading = pos.getHeading(AngleUnit.DEGREES);
            telemetry.addData("heading", heading);
            double newYawSpeed = bigYawSpeed;
            if (Math.abs(heading-placeHeading) < 2){
                newYawSpeed =  headingCorrectSpeed;
            }
            telemetry.update();
            if (heading > placeHeading + tolerance){
                yaw = newYawSpeed;
                rotate();
            }
            else if (heading < placeHeading - tolerance){
                yaw = -newYawSpeed;
                rotate();
            }
        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        claw.setPosition(CLAW_OPEN);
        sleep(100);


        odo.update();
        heading = pos.getHeading(AngleUnit.DEGREES);
        while (heading > moveHeading + tolerance || heading < moveHeading - tolerance){
            pos = odo.getPosition();
            odo.update();
            heading = pos.getHeading(AngleUnit.DEGREES);
            telemetry.addData("heading", heading);
            double newYawSpeed = bigYawSpeed;
            if (Math.abs(heading-moveHeading) < 2){
                newYawSpeed =  headingCorrectSpeed;
            }
            telemetry.update();
            if (heading > moveHeading + tolerance){
                yaw = newYawSpeed;
                rotate();
            }
            else if (heading < moveHeading - tolerance){
                yaw = -newYawSpeed;
                rotate();
            }
        }

    }

    public void pickupSpecial(){
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

        off();

//        go to specimen at wall
        while (pos.getX(DistanceUnit.INCH) > pickupX+distOff*blockNum && opModeIsActive()) {
            targetX = pickupX+distOff*blockNum;
            targetY = pickupY;

            pos = odo.getPosition();
            odo.update();

            double dist = Math.abs(pos.getX(DistanceUnit.INCH) - pickupX+distOff*blockNum);
            speed = Math.min(Math.log(dist/4+logAdd), maxSpeed);

            axial = -speed;
            lateral = 0;

            move();
        }
//        off();
        sleep(100);

//      grab specimen and bring it to place
        pickupClawAction();
    }

    //        bring specimen from pick up to appropriate distance on the wall
//       according to blockNum (how many specimens have already been placed)
    public void pickup(){
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

        //      back away from submersible
        if (blockNum > 1) {
            while (pos.getX(DistanceUnit.INCH) > 29 && opModeIsActive()) {
                pos = odo.getPosition();
                odo.update();

//            double dist = Math.abs(pos.getX(DistanceUnit.INCH) - 20);
//            speed = Math.min(Math.log(dist/4+logAdd), maxSpeed);

                targetX = 20;
                targetY = placementOffset * blockNum;

                axial = -speed;
                lateral = 0;

                move();
            }
        }
        off();

//      back away from submersible
        while (pos.getY(DistanceUnit.INCH) > pickupY && opModeIsActive()) {
            pos = odo.getPosition();
            odo.update();

//            double dist = Math.abs(pos.getX(DistanceUnit.INCH) - 20);
//            speed = Math.min(Math.log(dist/4+logAdd), maxSpeed);

            targetX = 20;
            targetY = placementOffset*blockNum;

            axial = -0.2;
            lateral = speed2;

            move();
        }
        off();
//        go to specimen at wall
        while (pos.getX(DistanceUnit.INCH) > pickupX+distOff*blockNum && opModeIsActive()) {
            targetX = pickupX+distOff*blockNum;
            targetY = pickupY;

            pos = odo.getPosition();
            odo.update();

            double dist = Math.abs(pos.getX(DistanceUnit.INCH) - pickupX+distOff*blockNum);
            speed = Math.min(Math.log(dist/4+logAdd), maxSpeed);

            axial = -speed;
            lateral = 0;

            move();
        }
        off();
        sleep(100);

//      grab specimen and bring it to place
        pickupClawAction();
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
        barl = hardwareMap.get(Servo.class, "brl");
        barr = hardwareMap.get(Servo.class, "brr");


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        leftRightHinge = hardwareMap.get(Servo.class, "hlr");
        telemetry.setMsTransmissionInterval(500);

        limelight.pipelineSwitch(1);
        limelight.start();


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
        odo.setOffsets(-65.0, -145.5); //these are tuned for 3110-0002-0001 Product Insight #1

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
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


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
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();


        while (opModeIsActive() && runtime.seconds() < 28.5) {

            telemetry.addData("Status", "Initialized");
            telemetry.addData("X offset", odo.getXOffset());
            telemetry.addData("Y offset", odo.getYOffset());
            telemetry.addData("Device Version Number:", odo.getDeviceVersion());
            telemetry.addData("Device Scalar", odo.getYawScalar());
            telemetry.update();

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

            sleep(10);

            if(!done) {
                // fluctuation test to see if our positions are messed up
                // fluctuationTest();
                bar_left.setPosition(0.61);
                bar_right.setPosition(0.55);
                left_right_hinge.setPosition(HINGE_LEFT);
                up_down_hinge.setPosition(0.2);
                // place preloaded specimen
                placeSpecimen();
                blockNum += 1;

                // go to first sample and drop it off at human player
                toSamp();
                dropOff();
                numSamp += 1;

                // go to second sample and drop it off at human player
                toSamp();
                dropOff();
                numSamp += 1; // update that now you need to go to the next sample

                // go to third sample and drop it off at human player
//                dropToSamp();
//                dropOff();

                // place second specimen
                pickupX += 6;
//                pickupX += 4;
                pickupSpecial(); // go from drop off of last sample to specimen pick up area
                placeSpecimen(); // place the second specimen
                blockNum += 1; // update how many blocks for alignment

                // place third specimen
                pickupY += 14;
                pickupX -= 6;
//                pickupX -= 5;
                pickup(); // go from submersible to specimen pick up area
                placeSpecimen(); // place the third specimen
                blockNum += 1; // update how many blocks for alignment

                pickupX = -1;

                // place fourth specimen
                pickup(); // go from submersible to specimen pick up area
//                placeSpecimen(); // place the fourth specimen
//                blockNum += 2.5; // update how many blocks for alignment
//
//                // place fifth specimen
//                pickup(); // go from submersible to specimen pick up area
//                placeSpecimen(); // place the fourth specimen
//                blockNum += 1; // update how many blocks for alignment

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