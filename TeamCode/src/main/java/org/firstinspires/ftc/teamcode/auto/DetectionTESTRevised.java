package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.JunctionDetectionPipelineRevised;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class DetectionTESTRevised extends LinearOpMode {
    JunctionDetectionPipelineRevised junctionDetectionPipelineRevised;
    OpenCvCamera camera;
    OpenCvCamera camera2;

    @Override
    public void runOpMode() {
        //SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        junctionDetectionPipelineRevised = new JunctionDetectionPipelineRevised();

        camera2.setPipeline(junctionDetectionPipelineRevised);
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
            if (junctionDetectionPipelineRevised.done) {
                telemetry.addLine("the distance is: " + junctionDetectionPipelineRevised.distance);
                telemetry.addLine(junctionDetectionPipelineRevised.started + " " + junctionDetectionPipelineRevised.middle + " " + junctionDetectionPipelineRevised.end);
            } else {
                telemetry.addLine("not done yet");
                telemetry.addLine(junctionDetectionPipelineRevised.started + " " + junctionDetectionPipelineRevised.middle + " " + junctionDetectionPipelineRevised.end);
            }
            telemetry.update();
            sleep(20);
        }
    }
}
