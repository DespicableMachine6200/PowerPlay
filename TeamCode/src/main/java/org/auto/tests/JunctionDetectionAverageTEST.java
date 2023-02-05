package org.auto.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.JunctionDetectionAVERAGE;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class JunctionDetectionAverageTEST extends LinearOpMode {
        JunctionDetectionAVERAGE JunctionDetectionPipeline;
        OpenCvCamera camera;
        OpenCvCamera camera2;

        @Override
        public void runOpMode() {
            //SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            camera2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
            JunctionDetectionPipeline = new JunctionDetectionAVERAGE();

            camera2.setPipeline(JunctionDetectionPipeline);
            camera2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera2.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });

            telemetry.setMsTransmissionInterval(50);
            telemetry.addLine("smth");
            telemetry.update();
            while (!isStarted() && !isStopRequested()) {
                if (JunctionDetectionPipeline.done) {
                    telemetry.addLine("the distance is: \n");
                    telemetry.addLine(JunctionDetectionPipeline.started + " " + JunctionDetectionPipeline.middle + " " + JunctionDetectionPipeline.end);
                } else {
                    telemetry.addLine("not done yet \n");
                    telemetry.addLine(JunctionDetectionPipeline.started + " " + JunctionDetectionPipeline.middle + " " + JunctionDetectionPipeline.end);
                }
                telemetry.update();
                sleep(20);
            }

            waitForStart();
            if(!isStopRequested()){
                telemetry.addLine("started \n");
                if (JunctionDetectionPipeline.done) {
                    telemetry.addLine("the distance is: \n");
                    telemetry.addLine(JunctionDetectionPipeline.started + " " + JunctionDetectionPipeline.middle + " " + JunctionDetectionPipeline.end);
                } else {
                    telemetry.addLine("not done yet \n");
                    telemetry.addLine(JunctionDetectionPipeline.started + " " + JunctionDetectionPipeline.middle + " " + JunctionDetectionPipeline.end);
                }
                telemetry.update();
                sleep(20);
            }
        }
    }

