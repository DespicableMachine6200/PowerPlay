package org.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.Team6200.TeamPID;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class PIDTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        TeamPID controller = new TeamPID(hardwareMap, robot, telemetry);
        waitForStart();
        controller.setTargetVelocity(1);
    }
}
