package org.auto.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.cameraReferenceIGNORE;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class cameraReferenceTEST extends LinearOpMode {
    cameraReferenceIGNORE cameraReference;
    OpenCvCamera camera;
    OpenCvCamera camera2;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        cameraReference = new cameraReferenceIGNORE();

        /*camera2.setPipeline(cameraReference);
        camera2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera2.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });*/

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("smth");
        telemetry.update();
        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("telemetry is working");
            /*if (!cameraReference.done) {
                telemetry.addLine("not done yet \n");
                telemetry.addLine(cameraReference.started + " " + cameraReference.middle + " " + cameraReference.end);
            } else {
                telemetry.addLine("the distance is \n");
                telemetry.addLine(cameraReference.started + " " + cameraReference.middle + " " + cameraReference.end);
            }*/
            telemetry.update();
            sleep(20);
        }

        /*waitForStart();
        if(!isStopRequested()){
            telemetry.addLine("started \n");
            if (cameraReference.done) {
                telemetry.addLine("the distance is: \n");
                telemetry.addLine(cameraReference.started + " " + cameraReference.middle + " " + cameraReference.end);
            } else {
                telemetry.addLine("not done yet \n");
                telemetry.addLine(cameraReference.started + " " + cameraReference.middle + " " + cameraReference.end);
            }
            telemetry.update();
            sleep(20);
        }*/
    }
}
