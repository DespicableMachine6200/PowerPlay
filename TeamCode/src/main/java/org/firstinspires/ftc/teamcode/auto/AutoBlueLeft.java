package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;

@Autonomous
public class AutoBlueLeft extends LinearOpMode {

    private DcMotor lmotor;

    //@Override
    /*public void init()
    {
        lmotor = hardwareMap.get(DcMotor.class, "linearSlide");
        lmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lmotor.setTargetPosition(0);


    }*/

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
    public void runOpMode() {
        float minPosition = 0.3f;
        float maxPosition = 0.8f;
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        lmotor = hardwareMap.get(DcMotor.class, "linearSlide");
        lmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lmotor.setTargetPosition(0);

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

        Pose2d startPose = new Pose2d(37, 60, Math.toRadians(270));
        robot.setPoseEstimate(startPose);

        TrajectorySequence seq1 = null;
        TrajectorySequence seq2 = null;
        TrajectorySequence seq3 = null;
        TrajectorySequence seq4 = null;
        TrajectorySequence seq5 = null;
        TrajectorySequence seq6 = null;

        if(tagOfInterest != null){
            if (tagOfInterest.id == LEFT) {
                seq1 = robot.trajectorySequenceBuilder(new Pose2d(35, 60, Math.toRadians(270)))
                        .turn(Math.toRadians(-90))
                        .forward(22)
                        .strafeLeft(36)
                        .build();
                seq2 = robot.trajectorySequenceBuilder(seq1.end())
                        // drop cone
                        .strafeLeft(12)
                        .turn(Math.toRadians(180))
                        .forward(50)
                        .build();
                seq3 = robot.trajectorySequenceBuilder(seq2.end())
                        // pick up cone
                        .back(42)
                        .turn(Math.toRadians(-90))
                        .build();
                seq4 = robot.trajectorySequenceBuilder(seq3.end())
                        // drop cone
                        .turn(Math.toRadians(90))
                        .forward(38)
                        .build();
                seq5 = robot.trajectorySequenceBuilder(seq4.end())
                        // pick up cone
                        .back(38)
                        .turn(Math.toRadians(-90))
                        .build();
                seq6 = robot.trajectorySequenceBuilder(seq5.end())
                        // drop cone
                        .strafeLeft(38)
                        .build();
                robot.followTrajectorySequence(seq1);
                robot.followTrajectorySequence(seq2);
                robot.followTrajectorySequence(seq3);
                robot.followTrajectorySequence(seq4);
                robot.followTrajectorySequence(seq5);
                robot.followTrajectorySequence(seq6);
            }
            else if (tagOfInterest.id == MIDDLE) {
                //insert trajectories for parking zone 2
                seq1 = robot.trajectorySequenceBuilder(new Pose2d(35, 60, Math.toRadians(270)))
                        .turn(Math.toRadians(-90))
                        .forward(22)
                        .strafeLeft(36)
                        .build();
                seq2 = robot.trajectorySequenceBuilder(seq1.end())
                        // drop cone
                        .strafeLeft(12)
                        .turn(Math.toRadians(180))
                        .forward(50)
                        .build();
                seq3 = robot.trajectorySequenceBuilder(seq2.end())
                        // pick up cone
                        .back(42)
                        .turn(Math.toRadians(-90))
                        .build();
                seq4 = robot.trajectorySequenceBuilder(seq3.end())
                        // drop cone
                        .turn(Math.toRadians(90))
                        .forward(38)
                        .build();
                seq5 = robot.trajectorySequenceBuilder(seq4.end())
                        // pick up cone
                        .back(38)
                        .turn(Math.toRadians(-90))
                        .build();
                seq6 = robot.trajectorySequenceBuilder(seq5.end())
                        // drop cone
                        .turn(Math.toRadians(90))
                        .forward(20)
                        .build();
                robot.followTrajectorySequence(seq1);
                robot.followTrajectorySequence(seq2);
                robot.followTrajectorySequence(seq3);
                robot.followTrajectorySequence(seq4);
                robot.followTrajectorySequence(seq5);
                robot.followTrajectorySequence(seq6);
            }

            else if (tagOfInterest.id == RIGHT) {
                //insert trajectories for parking zone 3
                seq1 = robot.trajectorySequenceBuilder(new Pose2d(35, 60, Math.toRadians(270)))
                        .turn(Math.toRadians(-90))
                        .forward(22)
                        .strafeLeft(36)
                        .build();
                seq2 = robot.trajectorySequenceBuilder(seq1.end())
                        // drop cone
                        .strafeLeft(12)
                        .turn(Math.toRadians(180))
                        .forward(50)
                        .build();
                seq3 = robot.trajectorySequenceBuilder(seq2.end())
                        // pick up cone
                        .back(42)
                        .turn(Math.toRadians(-90))
                        .build();
                seq4 = robot.trajectorySequenceBuilder(seq3.end())
                        // drop cone
                        .turn(Math.toRadians(90))
                        .forward(38)
                        .build();
                seq5 = robot.trajectorySequenceBuilder(seq4.end())
                        // pick up cone
                        .back(38)
                        .turn(Math.toRadians(-90))
                        .build();
                seq6 = robot.trajectorySequenceBuilder(seq5.end())
                        // drop cone
                        .turn(Math.toRadians(-90))
                        .forward(15)
                        .build();
                robot.followTrajectorySequence(seq1);
                robot.followTrajectorySequence(seq2);
                robot.followTrajectorySequence(seq3);
                robot.followTrajectorySequence(seq4);
                robot.followTrajectorySequence(seq5);
                robot.followTrajectorySequence(seq6);
            }
        }else{
            //failsafe trajectories
            seq1 = robot.trajectorySequenceBuilder(new Pose2d(35, 60, Math.toRadians(270)))
                    .turn(Math.toRadians(-90))
                    .forward(22)
                    .strafeLeft(36)
                    .build();
            seq2 = robot.trajectorySequenceBuilder(seq1.end())
                    // drop cone
                    .strafeLeft(12)
                    .turn(Math.toRadians(180))
                    .forward(50)
                    .build();
            seq3 = robot.trajectorySequenceBuilder(seq2.end())
                    // pick up cone
                    .back(42)
                    .turn(Math.toRadians(-90))
                    .build();
            seq4 = robot.trajectorySequenceBuilder(seq3.end())
                    // drop cone
                    .turn(Math.toRadians(90))
                    .forward(38)
                    .build();
            seq5 = robot.trajectorySequenceBuilder(seq4.end())
                    // pick up cone
                    .back(38)
                    .turn(Math.toRadians(-90))
                    .build();
            seq6 = robot.trajectorySequenceBuilder(seq5.end())
                    // drop cone
                    .turn(Math.toRadians(-90))
                    .forward(15)
                    .build();
            robot.followTrajectorySequence(seq1);
            robot.followTrajectorySequence(seq2);
            robot.followTrajectorySequence(seq3);
            robot.followTrajectorySequence(seq4);
            robot.followTrajectorySequence(seq5);
            robot.followTrajectorySequence(seq6);
        }

        waitForStart();
        if(!isStopRequested() && seq != null){
            robot.followTrajectorySequence(seq);
        }


    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));

    }
}