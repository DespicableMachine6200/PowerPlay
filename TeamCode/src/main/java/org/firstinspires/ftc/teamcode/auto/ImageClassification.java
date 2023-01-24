package org.firstinspires.ftc.teamcode.auto;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.CircleDetection;
import org.firstinspires.ftc.teamcode.vision.CircularTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.ColorDetection;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class ImageClassification extends LinearOpMode {

    private DcMotor lmotor;

    //@Override
    /*public void init()
    {
        lmotor = hardwareMap.get(DcMotor.class, "linearSlide");
        lmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lmotor.setTargetPosition(0);


    }*/

    ColorDetection circleDetection;
    OpenCvCamera camera;


    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    //don't change
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //our three tags
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;
    int tag = -1;

    @Override
    public void runOpMode() {
        float minPosition = 0.3f;
        float maxPosition = 0.8f;
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        circleDetection = new ColorDetection(hardwareMap, telemetry, 400, 224);
        lmotor = hardwareMap.get(DcMotor.class, "linearslide");
        lmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lmotor.setTargetPosition(0);

        camera.setPipeline(circleDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        while (!isStarted() && !isStopRequested()) {
            if (circleDetection.done && circleDetection.pos != -1) {
                if (circleDetection.pos == 0) {
                    telemetry.addLine("There was an error seeing the tag.");
                } else {
                    tag = circleDetection.pos;
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data: " + tag);
                    break;
                }
            } else {
                telemetry.addLine("Don't see tag of interest :(");
            }
            telemetry.update();
            sleep(20);
        }

        waitForStart();

    }
}
