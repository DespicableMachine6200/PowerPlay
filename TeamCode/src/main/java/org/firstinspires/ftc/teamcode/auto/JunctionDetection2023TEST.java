package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.JunctionDetection2023;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class JunctionDetection2023TEST extends LinearOpMode {
        JunctionDetection2023 junctionDetection13356;
        OpenCvCamera camera;
        OpenCvCamera camera2;

        @Override
        public void runOpMode() {
            //SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            camera2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
            junctionDetection13356 = new JunctionDetection2023();

            camera2.setPipeline(junctionDetection13356);
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

            while (!isStarted() && !isStopRequested()) {
                if (junctionDetection13356.done) {
                    telemetry.addLine("the distance is: " + junctionDetection13356.distance);
                    telemetry.addLine(junctionDetection13356.started + " " + junctionDetection13356.middle + " " + junctionDetection13356.end);
                } else {
                    telemetry.addLine("not done yet");
                    telemetry.addLine(junctionDetection13356.started + " " + junctionDetection13356.middle + " " + junctionDetection13356.end);
                }
                telemetry.update();
                sleep(20);
            }
        }
    }

