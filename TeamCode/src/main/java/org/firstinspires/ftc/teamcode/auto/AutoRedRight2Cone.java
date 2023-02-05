package org.firstinspires.ftc.teamcode.auto;

import android.graphics.RenderNode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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
public class  AutoRedRight2Cone extends LinearOpMode {
    public Servo servo;
    private DcMotor lmotor;

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

        Pose2d pos = new Pose2d(35.5, -63, Math.toRadians(180));
        robot.setPoseEstimate(pos);
        if(tagOfInterest != null){
            if (tagOfInterest.id == LEFT) {
                seq1 = robot.trajectorySequenceBuilder(new Pose2d(35.5, -63, Math.toRadians(90)))
                        // will this work idk
                        .setVelConstraint(robot.getVelocityConstraint(30, Math.toRadians(90), 15.02))
                        .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                        .forward(27.6)
                        .strafeRight(38.7)
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(3000))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setPower(1))
                        .forward(5.2)
                        .waitSeconds(1)
                        // drop cone 1
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        .waitSeconds(1)
                        .back(5)
                        .strafeLeft(7)
                        .turn(Math.toRadians(180))
                        .UNSTABLE_addTemporalMarkerOffset(-5, () -> robot.servo.setPosition(maxPosition))
                        .UNSTABLE_addTemporalMarkerOffset(-2, () -> robot.lmotor.setTargetPosition(571))
                        .forward(45)
                        // weird randomly shifts left while going forward could be a path contiuity error thing
                        /*.UNSTABLE_addTemporalMarkerOffset(-1.5, () -> robot.servo.setPosition(minPosition))
                        .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> robot.lmotor.setTargetPosition(700))
                        .back(36)
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(2950))
                        .turn(Math.toRadians(90))
                        .build();
                seq4 = robot.trajectorySequenceBuilder(seq3.end())
                     //rop cone
                        .forward(5)
                        // drop cone 2
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        .back(6)
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(maxPosition))
                        .turn(Math.toRadians(-90))
                        .forward(35)
                        .UNSTABLE_addTemporalMarkerOffset(-4, () -> robot.lmotor.setTargetPosition(400))
                        .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.servo.setPosition(minPosition))
                        .forward(6)
                        .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> robot.lmotor.setTargetPosition(700))
                        .back(36)
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(2950))
                        .turn(Math.toRadians(90))
                        .forward(5)
                        // drop cone 3
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        .back(6)
                        .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                        // park
                        .strafeRight(12)
                        .UNSTABLE_addTemporalMarkerOffset(-4, () -> robot.lmotor.setTargetPosition(0))*/
                        .build();
                        // spline stuff
                        /*.addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                        .strafeRight(20)
                        .lineToLinearHeading(new Pose2d(10, -24, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(-4, () -> robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION))
                        .UNSTABLE_addTemporalMarkerOffset(-4, () -> robot.lmotor.setTargetPosition(2950))
                        .UNSTABLE_addTemporalMarkerOffset(-4, () -> robot.lmotor.setPower(1))
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        // drop cone
                        .lineToLinearHeading(new Pose2d(24, -13, Math.toRadians(90)))
                        .UNSTABLE_addTemporalMarkerOffset(-3.5, () -> robot.servo.setPosition(maxPosition))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(571))
                        .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> robot.lmotor.setTargetPosition(700))
                        // pick up cone
                        .lineToLinearHeading(new Pose2d(24, -7, Math.toRadians(90)))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(2950))
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        // drop cone
                        .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(-3.5, () -> robot.servo.setPosition(maxPosition))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(400))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> robot.lmotor.setTargetPosition(700))
                        // pick up cone
                        .lineToLinearHeading(new Pose2d(24, -7, Math.toRadians(90)))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(2950))
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        // drop cone
                        // park
                        .lineToLinearHeading(new Pose2d(12, -13, Math.toRadians(90)))
                        .build();*/
            }

            else if (tagOfInterest.id == MIDDLE) {
                //insert trajectories for parking zone 2
                seq1 = robot.trajectorySequenceBuilder(new Pose2d(35.5, -63, Math.toRadians(90)))
                        .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                        .turn(Math.toRadians(90))
                        .forward(29)
                        .strafeLeft(37.6)
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(2950))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setPower(0.95))
                        .forward(4)
                        // drop cone 1
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        .back(6)
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(maxPosition))
                        .strafeLeft(7)
                        .turn(Math.toRadians(180))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(571))
                        .forward(52)
                        .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> robot.servo.setPosition(minPosition))
                        .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> robot.lmotor.setTargetPosition(700))
                        .back(36)
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(2950))
                        .turn(Math.toRadians(90))
                        .forward(5)
                        // drop cone 2
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        .back(6)
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(maxPosition))
                        .turn(Math.toRadians(-90))
                        .forward(35)
                        .UNSTABLE_addTemporalMarkerOffset(-4, () -> robot.lmotor.setTargetPosition(400))
                        .addTemporalMarker(-0.5, () -> robot.servo.setPosition(minPosition))
                        .forward(6)
                        .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> robot.lmotor.setTargetPosition(700))
                        .back(36)
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(2950))
                        .turn(Math.toRadians(90))
                        .forward(5)
                        // drop cone 3
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        .back(6)
                        .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                        // park
                        .strafeLeft(12)
                        .UNSTABLE_addTemporalMarkerOffset(-4, () -> robot.lmotor.setTargetPosition(0))
                        /* spline stuff
                        .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                        .strafeLeft(20)
                        .lineToLinearHeading(new Pose2d(10, -24, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(2950))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setPower(1))
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        // drop cone
                        .lineToLinearHeading(new Pose2d(24, -13, Math.toRadians(90)))
                        .UNSTABLE_addTemporalMarkerOffset(-3.5, () -> robot.servo.setPosition(maxPosition))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(571)))
                        .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition)))
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> robot.lmotor.setTargetPosition(700))
                        // pick up cone
                        .lineToLinearHeading(new Pose2d(24, -7, Math.toRadians(90)))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(2950))
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        // drop cone
                        .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(-3.5, () -> robot.servo.setPosition(maxPosition))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(400)))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> robot.lmotor.setTargetPosition(700))
                        // pick up cone
                        .lineToLinearHeading(new Pose2d(24, -7, Math.toRadians(90)))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(2950))
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        // drop cone
                        // park
                        .lineToLinearHeading(new Pose2d(36, -13, Math.toRadians(90)))*/
                        .build();
            }

            else if (tagOfInterest.id == RIGHT) {
                //insert trajectories for parking zone 3
                seq1 = robot.trajectorySequenceBuilder(new Pose2d(35.5, -63, Math.toRadians(90)))
                        .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                        .turn(Math.toRadians(90))
                        .forward(29)
                        .strafeLeft(37.6)
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(2950))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setPower(0.95))
                        .forward(4)
                        // drop cone 1
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        .back(6)
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(maxPosition))
                        .strafeLeft(7)
                        .turn(Math.toRadians(180))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(571))
                        .forward(52)
                        .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> robot.servo.setPosition(minPosition))
                        .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> robot.lmotor.setTargetPosition(700))
                        .back(36)
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(2950))
                        .turn(Math.toRadians(90))
                        .forward(5)
                        // drop cone 2
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        .back(6)
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(maxPosition))
                        .turn(Math.toRadians(-90))
                        .forward(35)
                        .UNSTABLE_addTemporalMarkerOffset(-4, () -> robot.lmotor.setTargetPosition(400))
                        .addTemporalMarker(-0.5, () -> robot.servo.setPosition(minPosition))
                        .forward(6)
                        .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> robot.lmotor.setTargetPosition(700))
                        .back(36)
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(2950))
                        .turn(Math.toRadians(90))
                        .forward(5)
                        // drop cone 3
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        .back(6)
                        .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                        // park
                        .strafeLeft(35)
                        .UNSTABLE_addTemporalMarkerOffset(-4, () -> robot.lmotor.setTargetPosition(0))
                        /* spline stuff
                        .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                        .strafeLeft(20)
                        .lineToLinearHeading(new Pose2d(10, -24, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(2950))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setPower(1))
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        // drop cone
                        .lineToLinearHeading(new Pose2d(24, -13, Math.toRadians(90)))
                        .UNSTABLE_addTemporalMarkerOffset(-3.5, () -> robot.servo.setPosition(maxPosition))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(571)))
                        .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition)))
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> robot.lmotor.setTargetPosition(700))
                        // pick up cone
                        .lineToLinearHeading(new Pose2d(24, -7, Math.toRadians(90)))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(2950))
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        // drop cone
                        .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(-3.5, () -> robot.servo.setPosition(maxPosition))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(400)))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> robot.lmotor.setTargetPosition(700))
                        // pick up cone
                        .lineToLinearHeading(new Pose2d(24, -7, Math.toRadians(90)))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(2950))
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        // drop cone
                        // park
                        .lineToLinearHeading(new Pose2d(60, -13, Math.toRadians(90)))*/
                        .build();
            }
        }else{
            //failsafe trajectories
            seq1 = robot.trajectorySequenceBuilder(new Pose2d(35.5, -63, Math.toRadians(180)))
                    .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                    .forward(27.6)
                    .strafeRight(38.1)
                    .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION))
                    .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(3000))
                    .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setPower(1))
                    .forward(5.2)
                    .waitSeconds(2.5)
                    // drop cone 1
                    .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                    .waitSeconds(1.5)
                    .back(9)
                    .strafeRight(15)
                    .turn(Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> robot.servo.setPosition(maxPosition))
                    .forward(50)
                    .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(571))
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                    .waitSeconds(2)
                    .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                    .waitSeconds(1)
                    .addTemporalMarker(() -> robot.lmotor.setTargetPosition(1000))
                    // use claw
                    .back(41)
                    .addTemporalMarker(() -> robot.lmotor.setTargetPosition(3100))
                    .turn(Math.toRadians(90))
                    .forward(5.2)
                    .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                    .waitSeconds(3.2)
                    .back(6)
                    .strafeLeft(7)
                    .addTemporalMarker(() -> robot.lmotor.setTargetPosition(0))
                    /*.UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(maxPosition))
                    // park
                    .strafeRight(12)
                    .UNSTABLE_addTemporalMarkerOffset(-4, () -> robot.lmotor.setTargetPosition(0)
                    //spline stuff
                    /*.setVelConstraint(robot.getVelocityConstraint(52.48291908330528, Math.toRadians(100), 15.02))
                        .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                        .strafeRight(32)
                        .lineToLinearHeading(new Pose2d(0.655, -47.687, Math.toRadians(180)))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(2950))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setPower(1))*/
                        /*.addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        // drop cone
                        .lineToLinearHeading(new Pose2d(24, -13, Math.toRadians(90)))
                        //.lineToLinearHeading(new Pose2d(47, -13, Math.toRadians(90)))
                        .UNSTABLE_addTemporalMarkerOffset(-3.5, () -> robot.servo.setPosition(maxPosition))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(571))
                        .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))
                        //.lineToLinearHeading(new Pose2d(11, -12, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> robot.lmotor.setTargetPosition(700))
                        // pick up cone
                        .lineToLinearHeading(new Pose2d(24, -7, Math.toRadians(90)))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(2950))
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        // drop cone
                        .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))
                        .UNSTABLE_addTemporalMarkerOffset(-3.5, () -> robot.servo.setPosition(maxPosition))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(400))
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> robot.lmotor.setTargetPosition(700))
                        // pick up cone
                        .lineToLinearHeading(new Pose2d(24, -7, Math.toRadians(90)))
                        .UNSTABLE_addTemporalMarkerOffset(-3, () -> robot.lmotor.setTargetPosition(2950))
                        .addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                        // drop cone
                        // park
                        .lineToLinearHeading(new Pose2d(12, -13, Math.toRadians(90)))*/
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