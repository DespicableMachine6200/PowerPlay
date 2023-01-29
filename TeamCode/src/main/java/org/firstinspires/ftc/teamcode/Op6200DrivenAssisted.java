package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.Team6200.Movement;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.JunctionDetectionPipelineRevised;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Op6200DrivenAssisted extends OpMode {
        JunctionDetectionPipelineRevised junctionDetectionPipelineRevised;
        float minPosition = 0.3f;
        float maxPosition = 0.8f;
        double minDistance = 5.0; //inches MUST CHANGE
        double maxDistance = 6.5; //inches MUST CHANGE
        double maxGo = 10.0; //inches MUST CHANGE

        private SampleMecanumDrive robot;
        OpenCvCamera camera2;


        @Override
        public void init() {

            //movement = new Movement(hardwareMap);
            robot = new SampleMecanumDrive(hardwareMap);
            robot.lmotor.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.lmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lmotor.setTargetPosition(0);
            robot.setPoseEstimate(new Pose2d(35.5, -63, Math.toRadians(90)));

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
        }

        @Override
        public void loop() {

            double drive = -gamepad1.left_stick_y/3.5;
            double strafe = (-gamepad1.left_stick_x * 1.1)/4.5;
            double turn = (gamepad1.right_stick_x)/4.9;


            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 0.65);
            double frontLeftPower = (drive + strafe + turn) / denominator;
            double backLeftPower = (drive - strafe + turn) / denominator;
            double frontRightPower = (drive - strafe - turn) / denominator;
            double backRightPower = (drive + strafe - turn) / denominator;

            robot.leftFront.setPower(frontLeftPower);
            robot.rightFront.setPower(frontRightPower);
            robot.leftRear.setPower(backLeftPower);
            robot.rightRear.setPower(backRightPower);


            //set servo position
            double gripPos = robot.servo.getPosition();
            if(gamepad1.left_trigger > 0 && gripPos > minPosition){
                gripPos = minPosition;
            }else if(gamepad1.right_trigger > 0 && gripPos < maxPosition){
                gripPos = maxPosition;
            }else{
                gripPos = gripPos;
            }

            //based on grip position, set position of servo
            robot.servo.setPosition(Range.clip(gripPos, minPosition, maxPosition));
            telemetry.addData("general servo position", robot.servo.getPosition());

            robot.update();
            //robot.updatePoseEstimate();
            Pose2d myPose = robot.getPoseEstimate();

            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());
            telemetry.update();


            //manual movement for linear slide
            int lmotorpos = robot.lmotor.getCurrentPosition();
            if(gamepad1.right_bumper)
            {
                robot.lmotor.setTargetPosition(lmotorpos + 250);
                robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lmotor.setPower(0.95);

            }

            if(gamepad1.left_bumper)
            {
                robot.lmotor.setTargetPosition(lmotorpos - 100);
                robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lmotor.setPower(0.95);


            }
            telemetry.addData("linear slide position", lmotorpos);

            if(gamepad1.triangle) {
                // high junction
                robot.lmotor.setTargetPosition(3000);
                robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lmotor.setPower(0.95);
            }
            if(gamepad1.x) {
                // medium
                robot.lmotor.setTargetPosition(2100);
                robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lmotor.setPower(0.95);
            }
            if(gamepad1.circle) {
                // low
                robot.lmotor.setTargetPosition(1400);
                robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lmotor.setPower(0.95);
            }
            if(gamepad1.a) {
                // ground
                robot.lmotor.setTargetPosition(250);
                robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lmotor.setPower(0.95);
            }

            // auto drop off cone - if you press dpad up and the robot is within a reasonable range it will move to the pole to drop
            if(gamepad1.dpad_up && junctionDetectionPipelineRevised.done && junctionDetectionPipelineRevised.distance <= maxGo) {
                if (junctionDetectionPipelineRevised.distance < minDistance) {
                    // go back
                    while (!(junctionDetectionPipelineRevised.distance > minDistance && junctionDetectionPipelineRevised.distance > maxDistance)) {
                        robot.leftFront.setPower(-0.75);
                        robot.rightFront.setPower(-0.75);
                        robot.leftRear.setPower(-0.75);
                        robot.rightRear.setPower(-0.75);
                    }
                } else {
                    // go forward
                    while (!(junctionDetectionPipelineRevised.distance > minDistance && junctionDetectionPipelineRevised.distance > maxDistance)) {
                        robot.leftFront.setPower(0.75);
                        robot.rightFront.setPower(0.75);
                        robot.leftRear.setPower(0.75);
                        robot.rightRear.setPower(0.75);
                    }
                }
            }

            // code to make gamepad rumble if you can drop a cone
            /*if(junctionDetectionPipelineRevised.done && junctionDetectionPipelineRevised.distance > minDistance && junctionDetectionPipelineRevised.distance > maxDistance) {
                gamepad1.rumble(10);
            }*/
        }
}
