
package org.firstinspires.ftc.teamcode;
//package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;
import static org.firstinspires.ftc.teamcode.Constants.*;

@TeleOp(name = "Limelight 3A Test", group = "Linear OpMode")
public class SensorLimelight3A extends LinearOpMode {

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
    private double blm = bld + 0.08;
    private double brm = brd - 0.08;

    @Override
    public void runOpMode() throws InterruptedException
    {
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
        waitForStart();

        while (opModeIsActive()) {
//            barl.setPosition(blm);
//            barr.setPosition(brm);
//
//            limelight.pipelineSwitch(0);
//            LLStatus status = limelight.getStatus();
//            telemetry.addData("Name", "%s",
//                    status.getName());
//            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                    status.getTemp(), status.getCpu(),(int)status.getFps());
//            telemetry.addData("Pipeline", "Index: %d, Type: %s",
//                    status.getPipelineIndex(), status.getPipelineType());
//
//            LLResult result = limelight.getLatestResult();
//            if (result != null) {
//                // Access general information
//                Pose3D botpose = result.getBotpose();
//                double captureLatency = result.getCaptureLatency();
//                double targetingLatency = result.getTargetingLatency();
//                double parseLatency = result.getParseLatency();
//                telemetry.addData("LL Latency", captureLatency + targetingLatency);
//                telemetry.addData("Parse Latency", parseLatency);
//
//                if (result.isValid()) {
//                    telemetry.addData("tx", result.getTx());
//                    telemetry.addData("txnc", result.getTxNC());
//                    telemetry.addData("ty", result.getTy());
//                    telemetry.addData("tync", result.getTyNC());
//
////                    if (result.getTx() > 0){
////                        top_arm.setPosition(0.1);// back side
////                    } else {
////                        top_arm.setPosition(0.8);
////                    }
//
//                    double arm_pos = mid_pos;
//                    arm_pos += result.getTx() * pos_rate;
//                    if (arm_pos < min_pos) {
//                        arm_pos = min_pos;
//                    } else if (arm_pos > max_pos) {
//                        arm_pos = max_pos;
//                    }
//
//                    leftRightHinge.setPosition(arm_pos);
//
//
//                    // Access color results
//                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
//                    for (LLResultTypes.ColorResult cr : colorResults) {
//                        telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
//                    }
//                }
//            }

            telemetry.addLine("opmode is active!!");
            if (gamepad2.right_stick_x > 0.2) {
                limelight.pipelineSwitch(0);
                leftRightHinge.setPosition(mid_pos);
                barl.setPosition(blm);
                barr.setPosition(brm);
                sleep(200);

                status = limelight.getStatus();
                telemetry.addData("Name", "%s",
                        status.getName());
                telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                        status.getTemp(), status.getCpu(), (int) status.getFps());
                telemetry.addData("Pipeline", "Index: %d, Type: %s",
                        status.getPipelineIndex(), status.getPipelineType());
                telemetry.update();

                result = limelight.getLatestResult();
                if (result != null) {
                    // Access general information
                    Pose3D botpose = result.getBotpose();
                    double captureLatency = result.getCaptureLatency();
                    double targetingLatency = result.getTargetingLatency();
                    double parseLatency = result.getParseLatency();
                    telemetry.addData("LL Latency", captureLatency + targetingLatency);
                    telemetry.addData("Parse Latency", parseLatency);

                    if (result.isValid()) {
                        telemetry.addData("tx", result.getTx());
                        telemetry.addData("txnc", result.getTxNC());
                        telemetry.addData("ty", result.getTy());
                        telemetry.addData("tync", result.getTyNC());

//                    if (result.getTx() > 0){
//                        top_arm.setPosition(0.1);// back side
//                    } else {
//                        top_arm.setPosition(0.8);
//                    }

                        double arm_pos = mid_pos;
                        arm_pos += result.getTx() * pos_rate;
                        if (arm_pos < min_pos) {
                            arm_pos = min_pos;
                        } else if (arm_pos > max_pos) {
                            arm_pos = max_pos;
                        }

                        leftRightHinge.setPosition(arm_pos);


                        // Access color results
                        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                        for (LLResultTypes.ColorResult cr : colorResults) {
                            telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                        }
                    }
                }
                sleep(200);
            }
            if (gamepad2.right_stick_x < -0.2){
                leftRightHinge.setPosition(mid_pos);
                limelight.pipelineSwitch(1);
                status = limelight.getStatus();
                telemetry.addData("Name", "%s",
                        status.getName());
                telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                        status.getTemp(), status.getCpu(),(int)status.getFps());
                telemetry.addData("Pipeline", "Index: %d, Type: %s",
                        status.getPipelineIndex(), status.getPipelineType());

                result = limelight.getLatestResult();
                if (result != null) {
                    // Access general information
                    Pose3D botpose = result.getBotpose();
                    double captureLatency = result.getCaptureLatency();
                    double targetingLatency = result.getTargetingLatency();
                    double parseLatency = result.getParseLatency();
                    telemetry.addData("LL Latency", captureLatency + targetingLatency);
                    telemetry.addData("Parse Latency", parseLatency);

                    if (result.isValid()) {
                        telemetry.addData("tx", result.getTx());
                        telemetry.addData("txnc", result.getTxNC());
                        telemetry.addData("ty", result.getTy());
                        telemetry.addData("tync", result.getTyNC());

//                    if (result.getTx() > 0){
//                        top_arm.setPosition(0.1);// back side
//                    } else {
//                        top_arm.setPosition(0.8);
//                    }

                        double arm_pos = mid_pos;
                        arm_pos += result.getTx() * pos_rate;
                        if (arm_pos < min_pos) {
                            arm_pos = min_pos;
                        } else if (arm_pos > max_pos) {
                            arm_pos = max_pos;
                        }

                        leftRightHinge.setPosition(arm_pos);


                        // Access color results
                        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                        for (LLResultTypes.ColorResult cr : colorResults) {
                            telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                        }
                    }
                }
                sleep(50);
            }
//            barl.setPosition(bld);
//            barr.setPosition(brd);

            telemetry.update();
        }
        limelight.stop();
    }

}