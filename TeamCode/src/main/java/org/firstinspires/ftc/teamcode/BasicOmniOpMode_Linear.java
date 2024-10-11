package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
@Disabled
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo intake1 = null;
    private Servo intake2 = null;
    private Servo intake3 = null;
    private Servo intake4 = null;
    private Servo intake5 = null;

    // Add variables for slow mode
    private boolean slowMode = false;
    private boolean previousBState = false;
    private final double SLOW_MODE_FACTOR = 0.5; // Adjust this value to change the slow mode speed

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake1 = hardwareMap.get(Servo.class, "intake_1");
        intake2 = hardwareMap.get(Servo.class, "intake_2");
        intake3 = hardwareMap.get(Servo.class, "intake_3");
        intake4 = hardwareMap.get(Servo.class, "intake_4");
        intake5 = hardwareMap.get(Servo.class, "intake_5");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized"); // print to control hub
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            // Check for slow mode toggle
            boolean currentBState = gamepad1.b;
            if (currentBState && !previousBState) {
                slowMode = !slowMode;
            }
            previousBState = currentBState;

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

            // Servo control (unchanged)
            if (gamepad2.a) intake1.setPosition(1);
            if (gamepad2.b) intake2.setPosition(1);
            if (gamepad2.x) intake3.setPosition(1);
            if (gamepad2.y) intake4.setPosition(1);
            if (gamepad2.dpad_up) intake5.setPosition(1);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Slow Mode", slowMode ? "Enabled" : "Disabled");
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}