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
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AutoRedRightFINAL extends LinearOpMode{

        AprilTagDetectionPipeline aprilTagDetectionPipeline;
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

        @Override
        public void runOpMode() throws InterruptedException {
            float minPosition = 0.3f;
            float maxPosition = 0.8f;
            SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
            //servo = hardwareMap.get(Servo.class, "servo" );

            robot.lmotor.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.lmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lmotor.setTargetPosition(0);

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

            camera.setPipeline(aprilTagDetectionPipeline);
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

            Pose2d pos = new Pose2d(35.5, -63, Math.toRadians(90));
            robot.setPoseEstimate(pos);
            if(tagOfInterest != null){
                if (tagOfInterest.id == LEFT) {
                    seq1 = robot.trajectorySequenceBuilder(new Pose2d(35.5, -63, Math.toRadians(90)))
                            // CONE 1 START
                            .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                            .forward(60.9) //50.9
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION))
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.lmotor.setTargetPosition(3000))
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.lmotor.setPower(1))
                            .back(10)
                            .strafeLeft(13.5) //12.5
                            .forward(6.6) //4.6
                            .waitSeconds(1)//prob make 0.5 cuz it works for cone 2
                            // drop cone 1
                            .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                            .waitSeconds(0.5) //take out/shorten
                            // CONE 1 END
                            // CONE 2 START
                            .back(11)
                            .lineToLinearHeading(new Pose2d(40, -11.5, Math.toRadians(0)))
                            .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> robot.servo.setPosition(maxPosition))
                            .forward(27)
                            .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(571))
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                            .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                            .waitSeconds(0.5) //take out?
                            .addTemporalMarker(() -> robot.lmotor.setTargetPosition(1000))
                            // use claw
                            .back(20)
                            .lineToLinearHeading(new Pose2d(26.2 , -10.6, Math.toRadians(90)))
                            .forward(5.9) //4.6
                            .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(3000))
                            .waitSeconds(1.5)
                            // drop cone 2
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                            .waitSeconds(0.5)
                            // CONE 2 END
                            // CONE 3 START
                            .back(11)
                            .lineToLinearHeading(new Pose2d(40, -11.5, Math.toRadians(0)))
                            .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> robot.servo.setPosition(maxPosition))
                            .forward(27)
                            .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(400))
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                            .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                            .waitSeconds(0.5) //take out?
                            .addTemporalMarker(() -> robot.lmotor.setTargetPosition(1000))
                            // use claw
                            .back(20)
                            .lineToLinearHeading(new Pose2d(28.3 , -10.6, Math.toRadians(90)))
                            .forward(4.6)
                            .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(3000))
                            .waitSeconds(1.5)
                            // drop cone 3
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                            .waitSeconds(0.5)
                            // CONE 3 END
                            // park
                            .back(5.6)
                            .strafeLeft(12)
                            .build();
                }

                else if (tagOfInterest.id == MIDDLE) {
                    //insert trajectories for parking zone 2
                    seq1 = robot.trajectorySequenceBuilder(new Pose2d(35.5, -63, Math.toRadians(90)))
                            // CONE 1 START
                            .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                            .forward(60.9) //50.9
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION))
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.lmotor.setTargetPosition(3000))
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.lmotor.setPower(1))
                            .back(10)
                            .strafeLeft(13.5) //12.5
                            .forward(6.6) //4.6
                            .waitSeconds(1)//prob make 0.5 cuz it works for cone 2
                            // drop cone 1
                            .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                            .waitSeconds(0.5) //take out/shorten
                            // CONE 1 END
                            // CONE 2 START
                            .back(11)
                            .lineToLinearHeading(new Pose2d(40, -11.5, Math.toRadians(0)))
                            .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> robot.servo.setPosition(maxPosition))
                            .forward(27)
                            .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(571))
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                            .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                            .waitSeconds(0.5) //take out?
                            .addTemporalMarker(() -> robot.lmotor.setTargetPosition(1000))
                            // use claw
                            .back(20)
                            .lineToLinearHeading(new Pose2d(26.2 , -10.6, Math.toRadians(90)))
                            .forward(5.9) //4.6
                            .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(3000))
                            .waitSeconds(1.5)
                            // drop cone 2
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                            .waitSeconds(0.5)
                            // CONE 2 END
                            // CONE 3 START
                            .back(11)
                            .lineToLinearHeading(new Pose2d(40, -11.5, Math.toRadians(0)))
                            .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> robot.servo.setPosition(maxPosition))
                            .forward(27)
                            .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(400))
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                            .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                            .waitSeconds(0.5) //take out?
                            .addTemporalMarker(() -> robot.lmotor.setTargetPosition(1000))
                            // use claw
                            .back(20)
                            .lineToLinearHeading(new Pose2d(27.8 , -10.6, Math.toRadians(90)))
                            .forward(4.6)
                            .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(3000))
                            .waitSeconds(1.5)
                            // drop cone 3
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                            .waitSeconds(0.5)
                            // CONE 3 END
                            // park
                            .back(5.6)
                            .strafeRight(16)
                            .build();
                }

                else if (tagOfInterest.id == RIGHT) {
                    //insert trajectories for parking zone 3
                    seq1 = robot.trajectorySequenceBuilder(new Pose2d(35.5, -63, Math.toRadians(90)))
                            // CONE 1 START
                            .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                            .forward(60.9) //50.9
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION))
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.lmotor.setTargetPosition(3000))
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.lmotor.setPower(1))
                            .back(10)
                            .strafeLeft(13.5) //12.5
                            .forward(6.6) //4.6
                            .waitSeconds(1)//prob make 0.5 cuz it works for cone 2
                            // drop cone 1
                            .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                            .waitSeconds(0.5) //take out/shorten
                            // CONE 1 END
                            // CONE 2 START
                            .back(11)
                            .lineToLinearHeading(new Pose2d(40, -11.5, Math.toRadians(0)))
                            .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> robot.servo.setPosition(maxPosition))
                            .forward(27)
                            .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(571))
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                            .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                            .waitSeconds(0.5) //take out?
                            .addTemporalMarker(() -> robot.lmotor.setTargetPosition(1000))
                            // use claw
                            .back(20)
                            .lineToLinearHeading(new Pose2d(26.2 , -10.6, Math.toRadians(90)))
                            .forward(5.9) //4.6
                            .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(3000))
                            .waitSeconds(1.5)
                            // drop cone 2
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                            .waitSeconds(0.5)
                            // CONE 2 END
                            // CONE 3 START
                            .back(11)
                            .lineToLinearHeading(new Pose2d(40, -11.5, Math.toRadians(0)))
                            .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> robot.servo.setPosition(maxPosition))
                            .forward(27)
                            .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(400))
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                            .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                            .waitSeconds(0.5) //take out?
                            .addTemporalMarker(() -> robot.lmotor.setTargetPosition(1000))
                            // use claw
                            .back(20)
                            .lineToLinearHeading(new Pose2d(28.3 , -10.6, Math.toRadians(90)))
                            .forward(4.6)
                            .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(3000))
                            .waitSeconds(1.5)
                            // drop cone 3
                            .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                            .waitSeconds(0.5)
                            // CONE 3 END
                            // park
                            .back(5.6)
                            .strafeRight(36)
                            .build();
                }
            }else{
                //failsafe trajectories
                seq1 = robot.trajectorySequenceBuilder(new Pose2d(35.5, -63, Math.toRadians(90)))
                        // CONE 1 START
                        .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                        .forward(60.9) //50.9
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.lmotor.setTargetPosition(3000))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.lmotor.setPower(1))
                        .back(10)
                        .strafeLeft(13.5) //12.5
                        .forward(6.6) //4.6
                        .waitSeconds(1)//prob make 0.5 cuz it works for cone 2
                        // drop cone 1
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        .waitSeconds(0.5) //take out/shorten
                        // CONE 1 END
                        // CONE 2 START
                        .back(11)
                        .lineToLinearHeading(new Pose2d(40, -11.5, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> robot.servo.setPosition(maxPosition))
                        .forward(27)
                        .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(571))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                        .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                        .waitSeconds(0.5) //take out?
                        .addTemporalMarker(() -> robot.lmotor.setTargetPosition(1000))
                        // use claw
                        .back(20)
                        .lineToLinearHeading(new Pose2d(26.2 , -10.6, Math.toRadians(90)))
                        .forward(5.9) //4.6
                        .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(3000))
                        .waitSeconds(1.5)
                        // drop cone 2
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                        .waitSeconds(0.5)
                        // CONE 2 END
                        // CONE 3 START
                        .back(11)
                        .lineToLinearHeading(new Pose2d(40, -11.5, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> robot.servo.setPosition(maxPosition))
                        .forward(27)
                        .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(400))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                        .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                        .waitSeconds(0.5) //take out?
                        .addTemporalMarker(() -> robot.lmotor.setTargetPosition(1000))
                        // use claw
                        .back(20)
                        .lineToLinearHeading(new Pose2d(28.3 , -10.6, Math.toRadians(90)))
                        .forward(4.6)
                        .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(3000))
                        .waitSeconds(1.5)
                        // drop cone 3
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                        .waitSeconds(0.5)
                        // CONE 3 END
                        // park
                        .back(5.6)
                        .strafeRight(16)
                        .build();

            }

            waitForStart();
            if(!isStopRequested() && seq1 != null){
                robot.followTrajectorySequence(seq1);
            }


        }

        void tagToTelemetry(AprilTagDetection detection)
        {
            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));

        }
    }


