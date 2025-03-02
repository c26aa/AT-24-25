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


import static org.firstinspires.ftc.teamcode.Constants.*;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Constants.*;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.Locale;

@Autonomous(name = "sample auto", group = "Linear OpMode")

public class SampleAuto extends LinearOpMode {
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;

    private int blockNum = 0;
    private double tolerance = 0.1;
    private double bucketX = 6.5;
    private int bucketY = 27;
    private double pickupX = bucketX+7;
    private double pickupY = 18.5;
    private int lift_top = 1320;
    private int lift_bottom = 0;
    private double placeHeading = -45;
    boolean moveStuff = true;
    boolean liftPosition = true;
    private double heading = 0.0;
    private double passTime = -1500;
    private double maxSpeed = 0.75;
    private double speed = maxSpeed;
    private double maxSpeed2 = 0.95;
    private double speed2 = maxSpeed2; // max value is 0.89
    private double slowSpeed = 0.3;
    private double correctionSpeed = 0.15;
    private double headingCorrectSpeed = 0.25;
    private double yawSpeed = 0.08;
    private double bigYawSpeed = 0.4;
    private double targetX = 0;
    private double logAdd = 1.1;
    private double axial = 0.0;
    private double lateral = 0.0;
    private double yaw = 0.0;
    boolean done = false;

    private Limelight3A limelight;
    private Servo leftRightHinge = null;
    private Servo barl = null;
    private Servo barr = null;
    final double max_pos = HINGE_LEFT;
    final double mid_pos = HINGE_MIDDLE;
    final double min_pos = HINGE_RIGHT;
    private double bld = 0.38;
    private double brd = 0.77;
    private double blm = bld + 0.18;
    private double brm = brd - 0.18;


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

    //    this function goes straight from the wall to place the specimen
    public void off(){
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        headingCorrect();
    }

    public void clawPickPos(){
        up_down_hinge.setPosition(WRIST_DOWN);
        claw.setPosition(CLAW_OPEN);
        bar_left.setPosition(0.42);
        bar_right.setPosition(0.73);
        sleep(800);
        bar_left.setPosition(0.383);
        bar_right.setPosition(0.767);
    }

    public void clawMidPos(){
//        bar_left.setPosition(0.44);
//        bar_right.setPosition(0.72);
        bar_left.setPosition(0.54);
        bar_right.setPosition(0.62);
        left_right_hinge.setPosition(HINGE_MIDDLE);
        up_down_hinge.setPosition(WRIST_MIDDLE);
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
//        telemetry.addData("heading", heading);
//        telemetry.addData("x", pos.getX(DistanceUnit.INCH));
//        telemetry.addData("y", pos.getY(DistanceUnit.INCH));
        telemetry.update();

        while (heading < -0.05 || heading > 0.05){
            pos = odo.getPosition();
            odo.update();
            heading = pos.getHeading(AngleUnit.DEGREES);

//            telemetry.addData("heading", heading);
//            telemetry.addData("x", pos.getX(DistanceUnit.INCH));
//            telemetry.addData("y", pos.getY(DistanceUnit.INCH));
//            telemetry.update();

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

    public void move(){
        Pose2D pos = odo.getPosition();
        odo.update();
        heading = pos.getHeading(AngleUnit.DEGREES);
//        telemetry.addData("x", pos.getX(DistanceUnit.INCH));
//        telemetry.addData("y", pos.getY(DistanceUnit.INCH));
//        telemetry.addData("heading", heading);
//        telemetry.update();
//        telemetry.addData("speed", speed);
//        telemetry.update();

        yaw = 0;
        if (heading > 0.1){
            yaw = yawSpeed;
        }
        if (heading < -0.1){
            yaw = -yawSpeed;
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

    public void limelight(){
        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();

        leftRightHinge.setPosition(mid_pos);
        barl.setPosition(blm);
        barr.setPosition(brm);
        up_down_hinge.setPosition(WRIST_MIDDLE);
        sleep(600);

//        telemetry.addData("Name", "%s",
//                status.getName());
//        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                status.getTemp(), status.getCpu(),(int)status.getFps());
//        telemetry.addData("Pipeline", "Index: %d, Type: %s",
//                status.getPipelineIndex(), status.getPipelineType());

        result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());

                double camHeight = 220;
                double cmx = camHeight * Math.tan(Math.toRadians(result.getTx()));
                double cmy = camHeight * Math.tan(Math.toRadians(result.getTy()));
//                        double cmx = leftRightHinge.getPosition()+gamepad2.right_stick_x*10;
//                        double cmy = gamepad2.left_stick_y*10;

                double clawHingeDist = 150;
                double hingeLeftRad = Math.toRadians(145);
                double hingeRightRad = Math.toRadians(35);
                double hingeAngleToServo = (HINGE_LEFT - HINGE_RIGHT) / (hingeLeftRad - hingeRightRad);
                double newX = Math.max(clawHingeDist * Math.cos(hingeLeftRad), Math.min(clawHingeDist * Math.cos(hingeRightRad), cmx));
                double newServoAngle = (Math.acos(newX / clawHingeDist) - hingeRightRad) * hingeAngleToServo + HINGE_RIGHT;
                double arm_pos = newServoAngle;

// right slides in (lower) is in
// left slides out (higher) is in

                telemetry.addData("dist y", cmy);
                telemetry.addData("distx", cmx);
                telemetry.addData("newX", newX);
                telemetry.addData("mid servo angle", Math.acos(newX / clawHingeDist));
                telemetry.addData("mid servo position", newServoAngle);
                telemetry.addData("mid servo real", leftRightHinge.getPosition());

                double projectedCamClawDist = 94 * Math.sin(Math.toRadians(55));
                //double changeY = cmy- projectedCamClawDist;
                double changeY = cmy + projectedCamClawDist - clawHingeDist * Math.sin(Math.acos(newX / clawHingeDist));

                telemetry.addData("desired change y", changeY);

                final double servoMaxAngle = Math.toRadians(180);
                final double servoMinAngle = Math.toRadians(77);
                final double servoToAngleFactor = (servoMaxAngle - servoMinAngle) / (RIGHT_SLIDES_OUT - RIGHT_SLIDES_IN);

                // Given values
                double slideRightPosition = slide_right.getPosition();

                // Convert servo position to radians
                double theta = (slideRightPosition - RIGHT_SLIDES_IN) * servoToAngleFactor + servoMinAngle;

                // Convert radians to current Y position
                double currentY = 242 * Math.cos(Math.asin(171 * Math.sin(theta) / 242)) - 171 * Math.cos(theta);

                // Compute new Y position
                double newFinal = currentY + changeY;

                // Compute new angle in radians
                double safeAcosInput = Math.max(-1, Math.min(1, (242 * 242 - newFinal * newFinal - 171 * 171) / (342 * newFinal)));
                double newAngle = Math.acos(safeAcosInput);
                telemetry.addData("Absolute Servo Angle", Math.toDegrees(newAngle));

                // Convert new angle back to servo position
                double next_posR = (newAngle - servoMinAngle) / servoToAngleFactor + RIGHT_SLIDES_IN;
                double servoChange = next_posR - slideRightPosition;
                double next_posL = slide_left.getPosition() - servoChange;

                telemetry.addData("calculated y",currentY);
                telemetry.addData("intended slide position", next_posR);

                double new_posL = Math.min(Math.max(next_posL, LEFT_SLIDES_IN), LEFT_SLIDES_OUT);
                double new_posR = Math.min(Math.max(next_posR, RIGHT_SLIDES_IN), RIGHT_SLIDES_OUT);

// double inAmount = 0.15; // lower will be more in, don't make less than 0 or greater than 0.6
// double leftPos = LEFT_SLIDES_OUT - inAmount; // left slides out is actually the in position
// double rightPos = RIGHT_SLIDES_IN + inAmount;
// slide_left.setPosition(leftPos);
// slide_right.setPosition(rightPos);
                if (new_posL > 0 && new_posR > 0 && new_posL < 1 && new_posR < 1) {
                    telemetry.addData("slide position", new_posL);
                    telemetry.addData("slide position", new_posR);
                    slide_left.setPosition(new_posL);
                    slide_right.setPosition(new_posR);
                } else {
                    telemetry.addData("slide position wrong", new_posL);
                    telemetry.addData("slide position wrong", new_posR);
                }

                up_down_hinge.setPosition(0);
                telemetry.update();

                new Thread(() -> {
                    leftRightHinge.setPosition(arm_pos);
                    // arm down
                    sleep(200);

                    claw.setPosition(CLAW_OPEN);
                    bar_left.setPosition(0.44);
                    bar_right.setPosition(0.71);
                }).start();

                up_down_hinge.setPosition(WRIST_DOWN);
                telemetry.update();
                sleep(200);
            } else {
                telemetry.addLine("Result is invalid");
                telemetry.update();
                limelight();
            }
        } else {
            telemetry.addLine("No limelight result");
            telemetry.update();
            axial = slowSpeed;
            lateral = 0;
            move();
            sleep(100);
            off();
            limelight();
        }
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


    // pick up sample and pass thru
    public void passThru(){
        double in_amount = 0.2;
        slide_left.setPosition(LEFT_SLIDES_IN+in_amount);
        slide_right.setPosition(RIGHT_SLIDES_OUT-in_amount);
        limelight();
        // position intake arm
        clawPickPos();
        sleep(400);

        // close claw
        claw.setPosition(0.91);
        sleep(600);

        // pass thru
        outtake_claw.setPosition(OUTTAKE_CLAW_OPEN);
        new Thread(() -> {
            top_arm.setPosition(OUTTAKE_ARM_BACK);//this line hasn't been tested, comment out if not working
            //sleep(400);
            claw.setPosition(CLAW_CLOSED-0.05);
            bar_left.setPosition(0.65);
            bar_right.setPosition(0.51);
            left_right_hinge.setPosition(HINGE_MIDDLE);
            up_down_hinge.setPosition(WRIST_UP);
            sleep(800);
            top_arm.setPosition(OUTTAKE_ARM_FRONT+0.06);
            sleep(200);
        }).start();

        double inAmount = 0.16; // lower will be more in, don't make less than 0 or greater than 0.6
        double leftPos = LEFT_SLIDES_OUT - inAmount; // left slides out is actually the in position
        double rightPos = RIGHT_SLIDES_IN + inAmount;
        slide_left.setPosition(leftPos);
        slide_right.setPosition(rightPos);

        new Thread(() -> {
            sleep(1500);
            outtake_claw.setPosition(OUTTAKE_CLAW_CLOSED);
            sleep(500);
            claw.setPosition(CLAW_OPEN);
            top_arm.setPosition(OUTTAKE_ARM_BACK);
            new Thread(() -> {
                sleep(800);
                double inA = 0.35; // lower will be more in, don't make less than 0 or greater than 0.6
                double lPos = LEFT_SLIDES_OUT - inA; // left slides out is actually the in position
                double rPos = RIGHT_SLIDES_IN + inA;
                slide_left.setPosition(lPos);
                slide_right.setPosition(rPos);
            }).start();
            sleep(500);
            top_arm.setPosition(OUTTAKE_ARM_FRONT);
        }).start();

        passTime = runtime.milliseconds();
//        sleep(2900);
        new Thread(() -> {
            sleep(2900);
            clawMidPos();
        }).start();

    }

    // place the sample
    public void place(){
        odo.update();

        double newTime = getRuntime();
        double loopTime = newTime - oldTime;
        oldTime = newTime;

        // Get the current Position (x & y in mm, and heading in degrees) of the robot
        Pose2D pos = odo.getPosition();
        odo.update();

        //    align with bucket front and back
        while (pos.getX(DistanceUnit.INCH) < bucketX - tolerance || pos.getX(DistanceUnit.INCH) > bucketX + tolerance && opModeIsActive()) {
            // Update the position
            pos = odo.getPosition();
            odo.update();

            double dist = Math.abs(pos.getX(DistanceUnit.INCH) - (bucketX));
            speed = maxSpeed;
            if (dist < 5){
                speed = correctionSpeed;
            }
            telemetry.update();
            if (pos.getX(DistanceUnit.INCH) < bucketX - tolerance){
                axial = speed;
                if (blockNum == 0){
                    lateral = -0.8;
                } else {
                    lateral = 0;
                }
                move();
            }
            if (pos.getX(DistanceUnit.INCH) > bucketX + tolerance){
                axial = -correctionSpeed-0.02;
                lateral = 0;
                move();
            }

            if (lift_left.getCurrentPosition() < lift_top && runtime.milliseconds() > passTime + 4000 && opModeIsActive()){
                lift_left.setPower(.8);
                lift_right.setPower(lift_left.getPower());
            } else {
                lift_left.setPower(0);
                lift_right.setPower(lift_left.getPower());
            }
        }
        off();

        pos = odo.getPosition();
        odo.update();
        //    align with bucket left and right
        if (blockNum < 4) {
            while (pos.getY(DistanceUnit.INCH) < bucketY - tolerance || pos.getY(DistanceUnit.INCH) > bucketY + tolerance && opModeIsActive()) {
                // Update the position
                pos = odo.getPosition();
                odo.update();

                targetX = bucketX;
                axial = 0;
                if (pos.getX(DistanceUnit.INCH) < targetX - tolerance) {
                    axial = correctionSpeed;
                } else if (pos.getX(DistanceUnit.INCH) > targetX + tolerance) {
                    axial = -correctionSpeed;
                }

                double dist = Math.abs(pos.getY(DistanceUnit.INCH) - (bucketY));
                if (dist < 5) {
                    speed2 = 0.25;
                } else {
                    speed2 = maxSpeed2;
                }
//            speed2 = Math.min(Math.log(dist/4+logAdd), maxSpeed2);

                if (pos.getY(DistanceUnit.INCH) < bucketY - tolerance) {
                    lateral = -speed2;
                    move();
                }
                if (pos.getY(DistanceUnit.INCH) > bucketY + tolerance) {
                    lateral = speed2;
                    move();
                }

                if (lift_left.getCurrentPosition() < lift_top && runtime.milliseconds() > passTime + 4000 && opModeIsActive()) {
                    lift_left.setPower(.8);
                    lift_right.setPower(lift_left.getPower());
                } else {
                    lift_left.setPower(0);
                    lift_right.setPower(lift_left.getPower());
                }
                // Update the position
                pos = odo.getPosition();
                odo.update();
            }
        }

        pos = odo.getPosition();
        odo.update();

        heading = pos.getHeading(AngleUnit.DEGREES);
        while (heading > placeHeading + tolerance || heading < placeHeading - tolerance){
            pos = odo.getPosition();
            odo.update();
            heading = pos.getHeading(AngleUnit.DEGREES);
//            telemetry.addData("heading", heading);
            double newYawSpeed = bigYawSpeed;
            if (Math.abs(heading-placeHeading) < 5){
                yaw = correctionSpeed;
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

            if (lift_left.getCurrentPosition() < lift_top && runtime.milliseconds() > passTime + 4000 && opModeIsActive()){
                lift_left.setPower(.8);
                lift_right.setPower(lift_left.getPower());
            } else {
                lift_left.setPower(0);
                lift_right.setPower(lift_left.getPower());
            }
        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);


        //  raise scissor lift
        while (lift_left.getCurrentPosition() < lift_top && opModeIsActive()) {
            lift_left.setPower(.8);
            lift_right.setPower(lift_left.getPower());
        }
        lift_left.setPower(0);
        lift_right.setPower(lift_left.getPower());

        axial = -slowSpeed;
        lateral = 0;
        move();
        if (blockNum < 2){
            sleep(300);
        } else {
            sleep(450);
        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);

        sleep(100);
        top_arm.setPosition(OUTTAKE_ARM_BUCKET);
        sleep(400);
        outtake_claw.setPosition(OUTTAKE_CLAW_OPEN);
        sleep(250);
        top_arm.setPosition(OUTTAKE_ARM_FRONT);
        sleep(500);

        axial = slowSpeed;
        lateral = 0;
        move();
        sleep(250);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);

        while (lift_left.getCurrentPosition() > lift_bottom && opModeIsActive()) {
            lift_left.setPower(-.7);
            lift_right.setPower(lift_left.getPower());
        }
        lift_left.setPower(0);
        lift_right.setPower(lift_left.getPower());
        top_arm.setPosition(OUTTAKE_ARM_BACK);

        odo.update();
        pos = odo.getPosition();
        heading = pos.getHeading(AngleUnit.DEGREES);
        clawMidPos();

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
        odo.update();

        while (pos.getY(DistanceUnit.INCH) < pickupY - tolerance || pos.getY(DistanceUnit.INCH) > pickupY + tolerance && opModeIsActive()) {
            // Update the position
            pos = odo.getPosition();
            odo.update();

            targetX = pickupX;
            axial = 0;
            if (pos.getX(DistanceUnit.INCH) < targetX - tolerance){
                axial = correctionSpeed+0.1;
            } else if (pos.getX(DistanceUnit.INCH) > targetX + tolerance){
                axial = -correctionSpeed;
            }



            double dist = Math.abs(pos.getY(DistanceUnit.INCH) - (pickupY));
            speed2 = maxSpeed2;
            if (dist < 2.5){
                speed2 = correctionSpeed;
            }

            if (pos.getY(DistanceUnit.INCH) < pickupY - tolerance){
                lateral = -speed2;
                move();
            }
            if (pos.getY(DistanceUnit.INCH) > pickupY + tolerance){
                lateral = speed2;
                move();
            }
            // Update the position
            pos = odo.getPosition();
            odo.update();

//            if (lift_left.getCurrentPosition() > lift_bottom){
//                lift_left.setPower(-.7);
//                lift_right.setPower(lift_left.getPower());
//            } else {
//                lift_left.setPower(0);
//                lift_right.setPower(lift_left.getPower());
//            }
        }
        off();

        while (pos.getX(DistanceUnit.INCH) < pickupX || lift_left.getCurrentPosition() > lift_bottom && opModeIsActive()) {
            speed = maxSpeed;
            if (pos.getX(DistanceUnit.INCH) < pickupX){
                axial = speed;
                lateral = 0;
                move();
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



        passThru();

//        heading = pos.getHeading(AngleUnit.DEGREES);
//        while (heading > 0 + tolerance || heading < 0 - tolerance){
//            if (heading > 0 + tolerance){
//                yaw = yawSpeed;
//                rotate();
//            } else if (heading < 0 - tolerance){
//                yaw = -yawSpeed;
//                rotate();
//            }
//        }

//        while (lift_left.getCurrentPosition() > lift_bottom && opModeIsActive()) {
//            if (lift_left.getCurrentPosition() > lift_bottom){
//                lift_left.setPower(-.7);
//                lift_right.setPower(lift_left.getPower());
//            } else {
//                lift_left.setPower(0);
//                lift_right.setPower(lift_left.getPower());
//            }
//            // Update the position
//            pos = odo.getPosition();
//            odo.update();
//        }
//        off();


//        while (pos.getY(DistanceUnit.INCH) < pickupY + pickupOffset*blockNum- tolerance || pos.getX(DistanceUnit.INCH) > pickupY + pickupOffset*blockNum + tolerance && opModeIsActive()){
//            if (pos.getY(DistanceUnit.INCH) < pickupY - tolerance){
//                strafeLeft();
//            }
//            if (pos.getY(DistanceUnit.INCH) > pickupY + tolerance){
//                strafeRight();
//            }
//        }
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

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();


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

       // double startTime = System.currentTimeMillis();

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
//        resetRuntime();
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < 28.5) {
//            telemetry.addData("Status", "Initialized");
//            telemetry.addData("X offset", odo.getXOffset());
//            telemetry.addData("Y offset", odo.getYOffset());
//            telemetry.addData("Device Version Number:", odo.getDeviceVersion());
//            telemetry.addData("Device Scalar", odo.getYawScalar());
//            telemetry.update();

            // Wait for the game to start (driver presses START)
           // waitForStart();

            // Run until the end of the match (driver presses STOP)

//            if (moveStuff) {
//                top_arm.setPosition(0.8);
//                outtake_claw.setPosition(.45);
//                moveStuff = false;
//            }


            // Request an update from the Pinpoint odometry computer
            odo.update();

            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            oldTime = newTime;

            // Get the current Position (x & y in mm, and heading in degrees) of the robot
            Pose2D pos = odo.getPosition();

            if(!done) {
                // Set servo positions
                slide_left.setPosition(LEFT_SLIDES_OUT);
                slide_right.setPosition(RIGHT_SLIDES_IN);
                outtake_claw.setPosition(OUTTAKE_CLAW_CLOSED);
                top_arm.setPosition(OUTTAKE_ARM_FRONT);
                clawMidPos();
                // place preloaded specimen
                place();
                blockNum += 1;
                bucketX += 1;
                bucketY -= 1;

                grab();
                place();
                blockNum += 1;
                pickupY += 8.5;
                pickupX += 0.5;
                bucketX += 0;
                bucketY += 1;

                grab();
                place();
                blockNum += 1;
                pickupY += 8.5;

                grab();
                place();
                blockNum += 1;
                done = true;
//                passThru();
            }


//            // Update telemetry
//            telemetry.addData("heading: ", pos.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("position: ", pos.getX(DistanceUnit.INCH));
//            telemetry.addData("Lift Left Power", lift_left.getPower());
//            telemetry.addData("Lift Right Power", lift_right.getPower());
//            telemetry.addData("Lift Left Position", lift_left.getCurrentPosition());
//            telemetry.addData("Lift Right Position", lift_right.getCurrentPosition());
//            telemetry.update();
//            resetRuntime();
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