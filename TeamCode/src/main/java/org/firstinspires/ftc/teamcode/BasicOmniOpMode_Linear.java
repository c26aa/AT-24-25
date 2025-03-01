package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;



@TeleOp(name = "Atomic Tele-Op", group = "Linear OpMode")
//@Disabled
public class BasicOmniOpMode_Linear extends LinearOpMode {

    //limelight stuff
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
    private boolean done = false;




    private double currentPosition = 0.8;
    private double currentPosition1 = 0.3; // Start the servo at the middle position
    private static final double CHANGE_AMOUNT = 0.005;
    private static final double CHANGE_AMOUNT1 = 0.002;
    public boolean useLiftEncoder = false;
    public int lift_target = 0;
    boolean resetLift = true;


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

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
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


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        leftRightHinge = hardwareMap.get(Servo.class, "hlr");
        barl = hardwareMap.get(Servo.class, "brl");
        barr = hardwareMap.get(Servo.class, "brr");

        telemetry.setMsTransmissionInterval(500);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();


        telemetry.addData("Status", "Initialized"); // print to control hub
        telemetry.update();

        telemetry.addData("Limelight Stream", "http://limelight.local:5800/stream.mjpg");
        telemetry.update();

        waitForStart();
        runtime.reset();



        while (opModeIsActive()) {
            if (!done){
                top_arm.setPosition(OUTTAKE_ARM_FRONT);
                done = true;
            }
            double max;


            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Apply slow mode factor if enabled
            if (slowMode) {
                axial *= SLOW_MODE_FACTOR;
                lateral *= SLOW_MODE_FACTOR;
                yaw *= SLOW_MODE_FACTOR;
            }


            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 0.7) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
//                slide_left.setPosition(0.8);// good up
//                slide_right.setPosition(0.9);//down
//                slide_right.setPosition(0.3);//good up positon
//                slide_left.setPosition(0.2);//down position

            //all gamepad 2 stuff
            // Servo control (unchanged) this sucks change this
            if (gamepad2.left_trigger > 0.1) {//closed positon
                currentPosition = slide_left.getPosition();
                currentPosition1 = slide_right.getPosition();

                currentPosition += CHANGE_AMOUNT;
                currentPosition = Math.min(Math.max(currentPosition, LEFT_SLIDES_IN), LEFT_SLIDES_OUT);//makes sure its never above or below min and max value
                slide_left.setPosition(currentPosition);
                currentPosition1 -= CHANGE_AMOUNT;
                currentPosition1 = Math.min(Math.max(currentPosition1, RIGHT_SLIDES_IN), RIGHT_SLIDES_OUT);
                slide_right.setPosition(currentPosition1);
            }

            if (gamepad2.right_trigger > 0.1) {//
                currentPosition = slide_left.getPosition();
                currentPosition1 = slide_right.getPosition();

                currentPosition -= CHANGE_AMOUNT;
                currentPosition = Math.min(Math.max(currentPosition, LEFT_SLIDES_IN), LEFT_SLIDES_OUT);//makes sure its never above or below min and max value
                slide_left.setPosition(currentPosition);
                currentPosition1 += CHANGE_AMOUNT;
                currentPosition1 = Math.min(Math.max(currentPosition1, RIGHT_SLIDES_IN), RIGHT_SLIDES_OUT);
                slide_right.setPosition(currentPosition1);

            }

            if (gamepad2.dpad_up || gamepad2.dpad_down) {
                useLiftEncoder = false;
            } else if (gamepad2.dpad_right || gamepad2.dpad_left) {
                useLiftEncoder = true;
            }

            if (useLiftEncoder) {
                if (gamepad2.dpad_right) {// set lift targets
                    lift_target = 470;
                    // Move lift up using encoders
                } else if (gamepad2.dpad_left) {
                    lift_target = 1410;
                    top_arm.setPosition(OUTTAKE_ARM_BUCKET);
                    // Move lift down using encoders
                }
                //NOTE TO SELF: IMPLEMENT MORE ROBUST CORRECTION MECHANISM
                if (lift_target > lift_left.getCurrentPosition() + 15) {
                    lift_left.setPower(SCISSORLIFT_POWER);
                } else if (lift_target < lift_left.getCurrentPosition() - 15) {
                    lift_left.setPower(-SCISSORLIFT_POWER);
                } else {
                    lift_left.setPower(0);
                }
            } else if (gamepad2.dpad_up && lift_left.getCurrentPosition() < 1410) {//scissor lift
                lift_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addLine("Switched to manual control mode.");
                lift_left.setPower(SCISSORLIFT_POWER);

                telemetry.addData("Left Lift Power", lift_left.getPower());
                telemetry.addData("Lift encoder value", lift_left.getCurrentPosition());
                telemetry.addData("Right Lift Power", lift_right.getPower());
                telemetry.update();
            } else if (gamepad2.dpad_down && lift_left.getCurrentPosition() > 30) {
                new Thread(() -> {
                    top_arm.setPosition(OUTTAKE_ARM_BACK);
                }).start();
                lift_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addLine("Switched to manual control mode.");
                double dist = Math.abs(lift_left.getCurrentPosition());
                double dist_speed = Math.log(dist/100 + 1.3);
                double new_slp = Math.max(Math.min(dist_speed, SCISSORLIFT_POWER), 0.3);
                lift_left.setPower(-new_slp);

                telemetry.addData("Left Lift Power", lift_left.getPower());
                telemetry.addData("Lift encoder value", lift_left.getCurrentPosition());
                telemetry.addData("Right Lift Power", lift_right.getPower());
                telemetry.update();
            } else {
                lift_left.setPower(0);
            }

            lift_right.setPower(lift_left.getPower());


            if (gamepad2.y) {//back position
                bar_left.setPosition(0.61);
                bar_right.setPosition(0.55);
                left_right_hinge.setPosition(HINGE_MIDDLE);
                up_down_hinge.setPosition(WRIST_UP);

            }

            if (gamepad2.b) {//regular pick up
//                bar_left.setPosition(0.383);
//                bar_right.setPosition(0.767);
                bar_left.setPosition(0.405);
                bar_right.setPosition(0.7405);
//                left_right_hinge.setPosition(HINGE_MIDDLE);
                up_down_hinge.setPosition(WRIST_DOWN);

            }

            if (gamepad2.a) {//claw middle
                bar_left.setPosition(0.44);
                bar_right.setPosition(0.72);
                left_right_hinge.setPosition(HINGE_MIDDLE);
                up_down_hinge.setPosition(WRIST_MIDDLE);
            }

            // PASS THRU
            if (gamepad2.x) {//handoff
                outtake_claw.setPosition(OUTTAKE_CLAW_OPEN);
                new Thread(() -> {
                    top_arm.setPosition(OUTTAKE_ARM_BACK);//this line hasn't been tested, comment out if not working
                    //sleep(400);
                    claw.setPosition(CLAW_CLOSED-0.06);
                    bar_left.setPosition(0.65);
                    bar_right.setPosition(0.51);
                    left_right_hinge.setPosition(HINGE_MIDDLE);
                    up_down_hinge.setPosition(WRIST_UP);
                    sleep(400);
                    top_arm.setPosition(OUTTAKE_ARM_FRONT+0.07);
                    sleep(200);
                }).start();

                double inAmount = 0.16; // lower will be more in, don't make less than 0 or greater than 0.6
                double leftPos = LEFT_SLIDES_OUT - inAmount; // left slides out is actually the in position
                double rightPos = RIGHT_SLIDES_IN + inAmount;
                slide_left.setPosition(leftPos);
                slide_right.setPosition(rightPos);

                new Thread(() -> {
                    sleep(1300);
                    outtake_claw.setPosition(OUTTAKE_CLAW_CLOSED);
                    sleep(500);
                    claw.setPosition(CLAW_OPEN);
                    top_arm.setPosition(OUTTAKE_ARM_BACK);
                    new Thread(() -> {
                        sleep(400);
                        double inA = 0.35; // lower will be more in, don't make less than 0 or greater than 0.6
                        double lPos = LEFT_SLIDES_OUT - inA; // left slides out is actually the in position
                        double rPos = RIGHT_SLIDES_IN + inA;
                        slide_left.setPosition(lPos);
                        slide_right.setPosition(rPos);
                    }).start();
                }).start();

            }


            if (gamepad2.right_bumper) {//close
                claw.setPosition(CLAW_CLOSED);//95
            }
            if (gamepad2.left_bumper) {//open
                claw.setPosition(CLAW_OPEN);

            }

            double gp2ly = -gamepad2.left_stick_y;
            double gp2lx = gamepad2.left_stick_x;
            telemetry.addData("input x",gp2lx);
            telemetry.addData("input y",gp2ly);

            if (gp2lx*gp2lx+gp2ly*gp2ly < -0.2 || gp2lx*gp2lx+gp2ly*gp2ly > 0.2) {
                double changeX = 3 * gp2lx;

                telemetry.addData("X changes", changeX);

                double clawHingeDist = 150;
                double hingeLeftRad = Math.toRadians(145);
                double hingeRightRad = Math.toRadians(35);
                double hingeAngleToServo = (HINGE_LEFT - HINGE_RIGHT) / (hingeLeftRad - hingeRightRad);
                double currentX = clawHingeDist*Math.cos((leftRightHinge.getPosition() - HINGE_RIGHT)/hingeAngleToServo + hingeRightRad);
                double newX = Math.max(clawHingeDist * Math.cos(hingeLeftRad), Math.min(clawHingeDist * Math.cos(hingeRightRad), currentX+changeX));
                double newServoAngle = (Math.acos(newX / clawHingeDist) - hingeRightRad) * hingeAngleToServo + HINGE_RIGHT;
                double arm_pos = newServoAngle;

                telemetry.addData("currentX", currentX);
                telemetry.addData("newX", newX);
                telemetry.addData("newServoAngle", newServoAngle);
                double changeY = 5 * gp2ly - 150 * Math.sin(Math.acos(newX / clawHingeDist)) + 150 * Math.sin(Math.acos(currentX / clawHingeDist));
                telemetry.addData("y change", changeY);
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
                //telemetry.addData("Absolute Servo Angle", Math.toDegrees(newAngle));

                leftRightHinge.setPosition(arm_pos);

                // Convert new angle back to servo position
                double next_posR = (newAngle - servoMinAngle) / servoToAngleFactor + RIGHT_SLIDES_IN;
                double servoChange = next_posR - slideRightPosition;
                double next_posL = slide_left.getPosition() - servoChange;

//                telemetry.addData("calculated y",currentY);
//                telemetry.addData("intended slide position", next_posR);

                double new_posL = Math.min(Math.max(next_posL, LEFT_SLIDES_IN), LEFT_SLIDES_OUT);
                double new_posR = Math.min(Math.max(next_posR, RIGHT_SLIDES_IN), RIGHT_SLIDES_OUT);
                if (new_posL > 0 && new_posR > 0 && new_posL < 1 && new_posR < 1) {
                    slide_left.setPosition(new_posL);
                    slide_right.setPosition(new_posR);
                }
            }



            int lift_encoder_value = lift_left.getCurrentPosition();
            int lift_encoder_value2 = lift_right.getCurrentPosition();

            //all game pad 1 stuff

            //             Check for slow mode toggle
            if (gamepad1.b && slowMode == false) {
                slowMode = !slowMode;
            }
            if (gamepad1.b && slowMode == true) {
                slowMode = !slowMode;
            }
//            if (gamepad1.a){//get ready to clip
//                top_arm.setPosition(0.7);
//                lift_left.setTargetPosition();// find right value
//                lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                telemetry.addLine("encoder mode inside function.");
//                lift_left.setPower(0.5);
//                while (lift_left.isBusy()) {
//                    sleep(10000000);
//                }
//                lift_left.setPower(0);
//                lift_right.setPower(0);
//                outtake_claw.setPosition(.2);
//
//
//            }

            if (gamepad1.left_bumper) {//open
                outtake_claw.setPosition(OUTTAKE_CLAW_OPEN);
            }

            if (gamepad1.right_bumper) {//close
                outtake_claw.setPosition(OUTTAKE_CLAW_CLOSED);
            }
            if (gamepad1.dpad_down) {//towards front/handoff
                top_arm.setPosition(OUTTAKE_ARM_FRONT);
                telemetry.addData("Top Arm Position", top_arm.getPosition());
                telemetry.update();
            }
            if (gamepad1.dpad_up) {
                top_arm.setPosition(OUTTAKE_ARM_BACK);// back side
                telemetry.addData("Top Arm Position", top_arm.getPosition());
                telemetry.update();
            }
            if (gamepad1.dpad_left) {
                top_arm.setPosition(OUTTAKE_ARM_BUCKET);//for bucket
                telemetry.addData("Top Arm Position", top_arm.getPosition());
                telemetry.update();
            }
            if (gamepad1.a) {
                useLiftEncoder = true;
                lift_target = lift_left.getCurrentPosition() + 300; // Set the lift target

                // Open the claw after a short delay (if needed)

                new Thread(() -> {
                    sleep(300);
                    top_arm.setPosition(OUTTAKE_ARM_CLIP);
                    sleep(1000); // Adjust this delay if necessary
                    outtake_claw.setPosition(OUTTAKE_CLAW_OPEN);
                    sleep(500); // Adjust this delay if necessary
                    top_arm.setPosition(OUTTAKE_ARM_BACK);
                    sleep(1000);
                    useLiftEncoder = true;
                    lift_target = 0;
                }).start();

            }
            if (gamepad1.y) {
                bar_left.setPosition(0.44);
                bar_right.setPosition(0.72);
                left_right_hinge.setPosition(HINGE_MIDDLE);
                up_down_hinge.setPosition(WRIST_MIDDLE);
                outtake_claw.setPosition(OUTTAKE_CLAW_CLOSED);
                new Thread(() -> {
                    sleep(300);
                    useLiftEncoder = true;
                    lift_target = 440;
                }).start();
                new Thread(() -> {
                    sleep(1000);
                    top_arm.setPosition(OUTTAKE_ARM_FRONT);
                }).start();



            }

            if (gamepad1.x && resetLift){
                lift_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addLine("Switched to manual control mode.");
                lift_left.setPower(-SCISSORLIFT_POWER);
                lift_right.setPower(lift_left.getPower());
                new Thread(() -> {
                    sleep(5000); // Adjust this delay if necessary
                    resetLift = false;
                    lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lift_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }).start();

            }


//            steps to make this work perfectly
//            1) make sure claw pickups sample when the sample is placed perfectly under the camera,
//                  if it too far in, make denominator of slidesToCM greater, and if too far our make denominator lower
//                  if it is perfect, then put a comment saying not to change slidesToCM
//            2) make sure claw picks up sample when the sample is in line with the claw, and the only thing that needs to happen is slides out, if it doesn't change what slide position is multiplied by
//            3) make sure claw picks up sample when it only takes a rotation of the hinge, but no slide movement, if it doesn't change pos_rate
//            4) make sure claw picks up when it takes both hinge and slides

//            if sample is picked up perfectly when directly under camera then:
//
            //limelight code
            if (gamepad2.right_stick_x > 0.2 || gamepad2.right_stick_x < -0.2) {
                if (gamepad2.right_stick_x > 0.2) {
                    limelight.pipelineSwitch(0);
                } else {
                    limelight.pipelineSwitch(1);
                }

                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);

                leftRightHinge.setPosition(mid_pos);
                barl.setPosition(blm);
                barr.setPosition(brm);
                up_down_hinge.setPosition(WRIST_MIDDLE);
                sleep(400);

                status = limelight.getStatus();
                telemetry.addData("Name", "%s", status.getName());
                telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d", status.getTemp(), status.getCpu(), (int) status.getFps());
                telemetry.addData("Pipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());

                result = limelight.getLatestResult();

                telemetry.addData("Result Type", limelight.getLatestResult().getClass().getName());
                telemetry.addData("status Type", limelight.getStatus().getClass().getName());
                telemetry.update();

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
                    }
                } else {
                    telemetry.addLine("No limelight result");
                    telemetry.update();
                }
            }

//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Slow Mode", slowMode ? "Enabled" : "Disabled");
//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//            telemetry.addData("encoder_value_left", lift_encoder_value);
            telemetry.update();

//            if (gamepad2.right_bumper) { // active intake code
//                // Spin the servo forward
//                claw.setPower(0.95); // Full forward speed
//            } else if (gamepad2.left_bumper) {
//                // Spin the servo backward
//                claw.setPower(-0.95); // Full reverse speed
//            } else {
//                // Stop the servo
//                claw.setPower(0.0); // Neutral position
//            }

        }
        limelight.stop();
    }
}