package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Constants.*;



@TeleOp(name = "Atomic Tele-Op", group = "Linear OpMode")
//@Disabled
public class BasicOmniOpMode_Linear extends LinearOpMode {

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

        telemetry.addData("Status", "Initialized"); // print to control hub
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
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
                currentPosition += CHANGE_AMOUNT;
                currentPosition = Math.min(Math.max(currentPosition, LEFT_SLIDES_IN), LEFT_SLIDES_OUT);//makes sure its never above or below min and max value
                slide_left.setPosition(currentPosition);
                currentPosition1 -= CHANGE_AMOUNT;
                currentPosition1 = Math.min(Math.max(currentPosition1, RIGHT_SLIDES_IN), RIGHT_SLIDES_OUT);
                slide_right.setPosition(currentPosition1);


            }

            if (gamepad2.right_trigger > 0.1) {//
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
                    lift_target = 430;
                    // Move lift up using encoders
                } else if (gamepad2.dpad_left) {
                    lift_target = 250;
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
            } else if (gamepad2.dpad_up && lift_left.getCurrentPosition() < 1400) {//scissor lift
                lift_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addLine("Switched to manual control mode.");
                lift_left.setPower(SCISSORLIFT_POWER);

                telemetry.addData("Left Lift Power", lift_left.getPower());
                telemetry.addData("Lift encoder value", lift_left.getCurrentPosition());
                telemetry.addData("Right Lift Power", lift_right.getPower());
                telemetry.update();
            } else if (gamepad2.dpad_down && lift_left.getCurrentPosition() > 30) {
                lift_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addLine("Switched to manual control mode.");
                lift_left.setPower(-SCISSORLIFT_POWER);

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
                bar_left.setPosition(0.38);
                bar_right.setPosition(0.77);
                left_right_hinge.setPosition(HINGE_MIDDLE);
                up_down_hinge.setPosition(WRIST_DOWN);

            }

            if (gamepad2.a) {//claw middle
                bar_left.setPosition(0.4);
                bar_right.setPosition(0.76);
                left_right_hinge.setPosition(HINGE_MIDDLE);
                up_down_hinge.setPosition(WRIST_MIDDLE);
            }
            if (gamepad2.x) {//handoff
                claw.setPosition(CLAW_CLOSED-0.02);
                outtake_claw.setPosition(OUTTAKE_CLAW_OPEN);
                bar_left.setPosition(0.65);
                bar_right.setPosition(0.51);
                left_right_hinge.setPosition(HINGE_MIDDLE);
                up_down_hinge.setPosition(0.0);
                top_arm.setPosition(OUTTAKE_ARM_FRONT+0.03);
                slide_left.setPosition(0.6);
                slide_right.setPosition(0.5);
                new Thread(() -> {
                    sleep(1000);
                    outtake_claw.setPosition(OUTTAKE_CLAW_CLOSED);
                    sleep(500);
                    claw.setPosition(CLAW_OPEN);
                    top_arm.setPosition(OUTTAKE_ARM_BACK);
                }).start();
            }


            if (gamepad2.right_bumper) {//close
                claw.setPosition(CLAW_CLOSED);//95
            }
            if (gamepad2.left_bumper) {//open
                claw.setPosition(CLAW_OPEN);

            }

            if (gamepad2.left_stick_x > 0.2){
                currentPosition += CHANGE_AMOUNT1;
                currentPosition = Math.min(Math.max(currentPosition, HINGE_RIGHT), HINGE_LEFT);//makes sure its never above or below min and max value
                left_right_hinge.setPosition(currentPosition);
            }
            if (gamepad2.left_stick_x < -0.2){
                currentPosition -= CHANGE_AMOUNT1;
                currentPosition = Math.min(Math.max(currentPosition, HINGE_RIGHT), HINGE_LEFT);//makes sure its never above or below min and max value
                left_right_hinge.setPosition(currentPosition);

            }


            int lift_encoder_value = lift_left.getCurrentPosition();
            int lift_encoder_value2 = lift_right.getCurrentPosition();

            //all game pad 1 stuff

            //             Check for slow mode toggle
            if (gamepad1.b && slowMode == false) {
                slowMode = !slowMode;
            }
            if (gamepad1.x && slowMode == true) {
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
            }
            if (gamepad1.dpad_up) {
                top_arm.setPosition(OUTTAKE_ARM_BACK);// back side

            }
            if (gamepad1.dpad_left) {
                top_arm.setPosition(OUTTAKE_ARM_BUCKET);//for bucket

            }
            if (gamepad1.a) {
                useLiftEncoder = true;
                lift_target = lift_left.getCurrentPosition() + 200; // Set the lift target

                // Open the claw after a short delay (if needed)
                new Thread(() -> {
                    sleep(500); // Adjust this delay if necessary
                    outtake_claw.setPosition(OUTTAKE_CLAW_OPEN);
                    sleep(1000); // Adjust this delay if necessary
                    top_arm.setPosition(OUTTAKE_ARM_BACK);
                }).start();
            }
            if (gamepad1.y) {
                bar_left.setPosition(0.4);
                bar_right.setPosition(0.76);
                left_right_hinge.setPosition(HINGE_MIDDLE);
                up_down_hinge.setPosition(WRIST_MIDDLE);
                outtake_claw.setPosition(CLAW_OPEN);
                useLiftEncoder = true;
                lift_target = lift_left.getCurrentPosition() + 130;
                sleep(1000);
                top_arm.setPosition(0.8);


            }

            if (gamepad1.dpad_right && resetLift){
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


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Slow Mode", slowMode ? "Enabled" : "Disabled");
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("encoder_value_left", lift_encoder_value);
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
    }
}
