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



@TeleOp(name = "Position Tester", group = "Linear OpMode")
//@Disabled
public class positionTester extends LinearOpMode {

    //limelight stuff
    private Limelight3A limelight;
    private Servo leftRightHinge = null;
    private Servo barl = null;
    private Servo barr = null;
    final double max_pos = HINGE_LEFT;
    final double mid_pos = HINGE_MIDDLE;
    final double min_pos = HINGE_RIGHT;
    final double pos_rate = -0.004;
    private double bld = 0.38;
    private double brd = 0.77;
    private double blm = bld + 0.13;
    private double brm = brd - 0.13;




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

        waitForStart();
        runtime.reset();

        bar_left.setPosition(0.54);
        bar_right.setPosition(0.62);
        left_right_hinge.setPosition(HINGE_MIDDLE);
        up_down_hinge.setPosition(WRIST_MIDDLE);

        while (opModeIsActive()) {
            double max;


            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double inAmount = 0.1;

            bar_left.setPosition(0.54);
            bar_right.setPosition(0.62);
//            left_right_hinge.setPosition(HINGE_MIDDLE);
            up_down_hinge.setPosition(WRIST_MIDDLE);

            // Apply slow mode factor if enabled
            if (gamepad2.y) {//back position
                double leftPos = LEFT_SLIDES_OUT - inAmount;
                double rightPos = RIGHT_SLIDES_IN + inAmount;
                slide_left.setPosition(leftPos);
                slide_right.setPosition(rightPos);
            }

            if (gamepad2.b) {//regular pick up
                double leftPos = LEFT_SLIDES_OUT - inAmount*2;
                double rightPos = RIGHT_SLIDES_IN + inAmount*2;
                slide_left.setPosition(leftPos);
                slide_right.setPosition(rightPos);
            }

            if (gamepad2.a) {//claw middle
                double leftPos = LEFT_SLIDES_OUT - inAmount*3;
                double rightPos = RIGHT_SLIDES_IN + inAmount*3;
                slide_left.setPosition(leftPos);
                slide_right.setPosition(rightPos);
            }

            if (gamepad2.x) {//claw middle
                double leftPos = LEFT_SLIDES_OUT - inAmount*4;
                double rightPos = RIGHT_SLIDES_IN + inAmount*4;
                slide_left.setPosition(leftPos);
                slide_right.setPosition(rightPos);
            }

            if (gamepad1.y) {//back position
                double leftPos = HINGE_LEFT - inAmount;
                leftRightHinge.setPosition(leftPos);
            }

            if (gamepad1.b) {//regular pick up
                double leftPos = HINGE_LEFT - inAmount*2;
                leftRightHinge.setPosition(leftPos);
            }

            if (gamepad1.a) {//claw middle
                double leftPos = HINGE_LEFT - inAmount*3;
                leftRightHinge.setPosition(leftPos);
            }

            if (gamepad1.x) {//claw middle
                double leftPos = HINGE_LEFT - inAmount*4;
                leftRightHinge.setPosition(leftPos);
            }

        }
        limelight.stop();
    }
}
