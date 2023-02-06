package org.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.Team6200.LinearSlidePID;
import org.Team6200.NewPID;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class LinearSlidePIDTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        robot.lmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        NewPID pid = new NewPID(telemetry, hardwareMap, robot);
        waitForStart();
        pid.setTargetPosition(1500);
    }
}
