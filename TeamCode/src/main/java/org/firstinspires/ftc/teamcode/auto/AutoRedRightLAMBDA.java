package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.JunctionDetectionPipelineRevised;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

@Autonomous
public class AutoRedRightLAMBDA extends LinearOpMode {
        AprilTagDetectionPipeline aprilTagDetectionPipeline;
        JunctionDetectionPipelineRevised junctionDetectionPipelineRevised;
        WebcamName camera;
        WebcamName camera2;
        OpenCvSwitchableWebcam switchableWebcam;

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

        @Override
        public void runOpMode() throws InterruptedException {
            float minPosition = 0.3f;
            float maxPosition = 0.8f;
            SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
            //servo = hardwareMap.get(Servo.class, "servo" );

            robot.lmotor.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.lmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lmotor.setTargetPosition(0);

            camera = hardwareMap.get(WebcamName.class, "Webcam 1");
            camera2 = hardwareMap.get(WebcamName.class, "Webcam 2");

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, camera, camera2);

            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
            junctionDetectionPipelineRevised = new JunctionDetectionPipelineRevised();

            switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    switchableWebcam.setPipeline(aprilTagDetectionPipeline);
                    switchableWebcam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode){}
            });

            telemetry.setMsTransmissionInterval(50);


            while (!isStarted() && !isStopRequested()) {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLD();

                if (currentDetections.size() != 0) {
                    boolean tagFound = false;

                    for (AprilTagDetection tag : currentDetections) {
                        if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }

                    if (tagFound) {
                        telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                        tagToTelemetry(tagOfInterest);
                    } else {
                        telemetry.addLine("no tag boi");
                    }

                } else {
                    telemetry.addLine("Don't see tag of interest :(");


                }

                telemetry.update();
                sleep(20);
            }


            if (tagOfInterest != null) {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            } else {
                telemetry.addLine("No tag snapshot available, never sighted(");
                telemetry.update();
            }
            TrajectorySequence seq1 = null;
            AtomicReference<TrajectorySequence> seq2 = null;
            TrajectorySequence seq3 = null;

            Pose2d pos = new Pose2d(35.5, -63, Math.toRadians(90));
            robot.setPoseEstimate(pos);
            if(tagOfInterest != null){
                seq1 = robot.trajectorySequenceBuilder(new Pose2d(35.5, -63, Math.toRadians(90)))
                        .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                        .turn(Math.toRadians(90))
                        .forward(27.6)
                        .strafeLeft(34.15)
                        .addTemporalMarker(() -> robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION))
                        .addTemporalMarker(() -> robot.lmotor.setTargetPosition(3030))
                        .addTemporalMarker(() -> robot.lmotor.setPower(1))
                        .waitSeconds(4)
                        .addTemporalMarker(()->{
                            seq2.set(robot.trajectorySequenceBuilder(new Pose2d(35.5, -63, Math.toRadians(90)))
                                    .forward(smth.getDistance()) // may be 0
                                    .build());
                        })
                        .waitSeconds(1.5)
                        // drop cone 1
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        .waitSeconds(1.5)
                        .back(9)
                        .build();
                seq3 = robot.trajectorySequenceBuilder(seq2.end())
                        //after stuff
                        .build();
            }

            waitForStart();
            if(!isStopRequested() && seq1 != null){
                switchableWebcam.setActiveCamera(camera2);
                switchableWebcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                switchableWebcam.setPipeline(junctionDetectionPipelineRevised);
                robot.followTrajectorySequence(seq1);
                robot.followTrajectorySequence(seq2);
                robot.followTrajectorySequence(seq3);
            }


        }

        void tagToTelemetry(AprilTagDetection detection)
        {
            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));

        }
    }

