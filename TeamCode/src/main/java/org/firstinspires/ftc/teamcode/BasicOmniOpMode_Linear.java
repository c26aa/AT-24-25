package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Atomic Tele-Op", group="Linear OpMode")
//@Disabled
public class BasicOmniOpMode_Linear extends LinearOpMode {

    private double currentPosition = 0.45;
    private double currentPosition1 = 0.4; // Start the servo at the middle position
    private static final double CHANGE_AMOUNT = 0.02;
    private final double claw_start = 0.5;
    private double claw_target = 0.5;
    private static final double claw_speed = 0.01;

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
    private Servo bucket = null;
    private DcMotor lift_left = null;
    private DcMotor lift_right = null;

    // Add variables for slow mode
    private boolean slowMode = false;
    private final double SLOW_MODE_FACTOR = 0.5; // Adjust this value to change the slow mode speed

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        lift_left = hardwareMap.get(DcMotor.class, "scl"); //left lift
        lift_right = hardwareMap.get(DcMotor.class, "scr"); // right lift
        slide_left = hardwareMap.get(Servo.class, "sll");
        slide_right = hardwareMap.get(Servo.class, "slr");
        bar_left = hardwareMap.get(Servo.class, "brl");
        bar_right = hardwareMap.get(Servo.class, "brr");
        left_right_hinge = hardwareMap.get(Servo.class, "hlr");
        up_down_hinge = hardwareMap.get(Servo.class, "hup");
        claw = hardwareMap.get(Servo.class, "clw");
        bucket = hardwareMap.get(Servo.class, "bkt");


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
        lift_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized"); // print to control hub
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

//             Check for slow mode toggle
            if (gamepad1.b) {
                slowMode = !slowMode;
            }
            if (gamepad1.x)
            {
                slowMode = !slowMode;
            }
            if(gamepad1.left_bumper){
bucket.setPosition(.6);
            }

            if(gamepad1.right_bumper){
bucket.setPosition(.99);
            }

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Apply slow mode factor if enabled
            if (slowMode) {
                axial *= SLOW_MODE_FACTOR;
                lateral *= SLOW_MODE_FACTOR;
                yaw *= SLOW_MODE_FACTOR;
            }




            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Servo control (unchanged) this sucks change this
            if (gamepad2.left_trigger > 0.1){
                currentPosition += CHANGE_AMOUNT;
                currentPosition = Math.min(Math.max(currentPosition, 0.45), 0.8);//makes sure its never above or below min and max value
                slide_left.setPosition(currentPosition);
                currentPosition1 -= CHANGE_AMOUNT;
                currentPosition1 = Math.min(Math.max(currentPosition1, 0.1), 0.4);
                slide_right.setPosition(currentPosition1);
                sleep(100);// 100 ms delay each time
            }

            if (gamepad2.right_trigger > 0.1){
                currentPosition -= CHANGE_AMOUNT;
                currentPosition = Math.min(Math.max(currentPosition, 0.45), 0.8);//makes sure its never above or below min and max value
                slide_left.setPosition(currentPosition);
                currentPosition1 += CHANGE_AMOUNT;
                currentPosition1 = Math.min(Math.max(currentPosition1, 0.1), 0.4);
                slide_right.setPosition(currentPosition1);
                sleep(100);// 100 ms delay each time
            }
            double lift_power = 0.2;//95
        int lift_max_right = 3000;//not true
        int lift_min_right = 3000;//not true
            int lift_max_left = 3000;//not true
            int lift_min_left = 3000;//not true
            if (gamepad2.dpad_up) {//scissor lift
                lift_left.setPower(lift_power);
//                lift_left.setTargetPosition(lift_max);
//                lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift_right.setPower(-lift_power);
//                lift_right.setTargetPosition(lift_max);
//                lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.dpad_down) {
                lift_left.setPower(-lift_power);
//                lift_left.setTargetPosition(lift_min);
//                lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift_right.setPower(lift_power);
//                lift_right.setTargetPosition(lift_min);
//                lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(gamepad2.y) {//back position
                bar_left.setPosition(1.0);
                bar_right.setPosition(1.0);
                left_right_hinge.setPosition(0.6);
                up_down_hinge.setPosition(0.27);
                claw.setPosition(0.5);
            }

            if(gamepad2.b) {//pick up from top
//                bar_left.setPosition(null);
//                bar_right.setPosition(null);
//                left_right_hinge.setPosition(0.6);
//                up_down_hinge.setPosition(0.35);
//                claw.setPosition(0.5);
            }

            if(gamepad2.a) {// pick up from side
//                bar_left.setPosition(null);
//                bar_right.setPosition(null);
//                left_right_hinge.setPosition(0.6);
//                up_down_hinge.setPosition(0.42);
//                claw.setPosition(0.5);
            }

            double leftStickX = gamepad1.left_stick_x;
            claw_target = 0.5 + Math.abs((leftStickX * 0.1));
            if (gamepad2.left_stick_x > 0 || gamepad2.left_stick_x < 0 ){
                claw.setPosition(claw_target);
            }
            if (gamepad2.right_stick_x > 0 ){
                left_right_hinge.setPosition(0.55);
            }
            else if (gamepad2.right_stick_x < 0 ) {
                left_right_hinge.setPosition(0.65);
            }
            int lift_encoder_value = lift_left.getCurrentPosition();
            int lift_encoder_value2 = lift_right.getCurrentPosition();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Slow Mode", slowMode ? "Enabled" : "Disabled");
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("encoder_value_left", lift_encoder_value);
            telemetry.addData("encoder_value_right", lift_encoder_value2);
            telemetry.update();
        }
    }
}
