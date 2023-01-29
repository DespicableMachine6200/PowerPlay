package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.JunctionDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous
public class AutoRedRghtTemporal2 extends LinearOpMode {
    // as this is a test april tags are not used
    WebcamName camera;
    WebcamName camera2;
    OpenCvCamera camera2CV;
    JunctionDetectionPipeline pl = null;
    @Override
    public void runOpMode() throws InterruptedException {
        float minPosition = 0.3f;
        float maxPosition = 0.8f;
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        robot.lmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lmotor.setTargetPosition(0);

        camera = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        camera2CV = setupCamera(camera2.getDeviceName(), new JunctionDetectionPipeline());

        AtomicReference<Double> forwardToJunction1 = new AtomicReference<>((double) 0);
        TrajectorySequence seq = robot.trajectorySequenceBuilder(new Pose2d(35.5, -63, Math.toRadians(90)))
                .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                .turn(Math.toRadians(90))
                .forward(27.6)
                .strafeLeft(34.15)
                .addTemporalMarker(() -> robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION))
                .addTemporalMarker(() -> robot.lmotor.setTargetPosition(3030))
                .addTemporalMarker(() -> robot.lmotor.setPower(1))
                .waitSeconds(4)
                .addTemporalMarker(()->{
                    forwardToJunction1.set(getDistance(camera2CV));
                    telemetry.addData(forwardToJunction1.get() + "", "");
                })
                .waitSeconds(0.5)
                .forward(forwardToJunction1.get())
                .waitSeconds(1.5)
                // drop cone 1
                .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                .waitSeconds(1.5)
                .back(0-forwardToJunction1.get())
                .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                .turn(Math.toRadians(-90))
                .forward(14.4)
                .UNSTABLE_addTemporalMarkerOffset(-3.5, () -> robot.lmotor.setTargetPosition(0))
                .build();

    }

    OpenCvCamera setupCamera(String cameraName, JunctionDetectionPipeline pipeline){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, cameraName), cameraMonitorViewId);

        camera.setPipeline(pipeline);
        pl = pipeline;
        return camera;
    }

    Double getDistance(OpenCvCamera camera){
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        while(!pl.done){
            telemetry.addData(pl.middle + "", ":"+pl.end);
            telemetry.update();
        }
        return pl.distance;
    }
}
