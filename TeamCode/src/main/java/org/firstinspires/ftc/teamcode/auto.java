package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



    /**
     * This file illustrates the concept of driving a path based on encoder counts.
     * The code is structured as a LinearOpMode
     *
     * The code REQUIRES that you DO have encoders on the wheels,
     *   otherwise you would use: RobotAutoDriveByTime;
     *
     *  This code ALSO requires that the drive Motors have been configured such that a positive
     *  power command moves them forward, and causes the encoders to count UP.
     *
     *   The desired path in this example is:
     *   - Drive forward for 48 inches
     *   - Spin right for 12 Inches
     *   - Drive Backward for 24 inches
     *   - Stop and close the claw.
     *
     *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
     *  that performs the actual movement.
     *  This method assumes that each movement is relative to the last stopping place.
     *  There are other ways to perform encoder based moves, but this method is probably the simplest.
     *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
     *
     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
     */

    @Autonomous(name="auto TEST PARK")

    public class auto extends LinearOpMode {

        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor leftFrontDrive = null;
        private DcMotor leftBackDrive = null;
        private DcMotor rightFrontDrive = null;
        private DcMotor rightBackDrive = null;



        private Thread telemetryH = new Thread() {
            @Override
            public void run() {
                while (opModeInInit() || opModeIsActive()) {

                    telemetry.update();
                }
            }
        };


        //TODO: TOUCH SENSOR + ADD IN APRIL TAGS + INIT
        @Override
        public void runOpMode() {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            double startTime = System.currentTimeMillis();

            // Initialize the drive system variables.
            leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
            leftBackDrive  = hardwareMap.get(DcMotor.class, "lb");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
            rightBackDrive = hardwareMap.get(DcMotor.class, "rb");


            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            // Send telemetry message to indicate successful Encoder reset
            telemetryH.start();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            while (opModeIsActive()) {
                leftFrontDrive.setPower(-.2);
                leftBackDrive.setPower(-.2);
                rightBackDrive.setPower(-.2);
                rightFrontDrive.setPower(-.2);


                telemetry.addData("Path", "Complete");
                telemetry.addData("TIME", startTime - getRuntime());
                telemetry.update();
                sleep(1000);  // pause to display final telemetry message.
            }
        }
}

