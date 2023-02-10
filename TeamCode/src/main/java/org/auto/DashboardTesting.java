package org.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class DashboardTesting extends OpMode {
        /*public DashboardPID(Telemetry telem, HardwareMap hwm, SampleMecanumDrive rbt){
            telemetry = telem;
            hardwareMap = hwm;
            robot = rbt;
            linearSlide = rbt.lmotor;
            startLoop();
        }*/
        public static double K_p = 0.5;
        public static double K_i = 0.01;
        public static double K_d = 0.1;
        public double desiredSetpoint;
        Telemetry telemetry;
        HardwareMap hardwareMap;
        SampleMecanumDrive robot;
        DcMotor linearSlide;

        @Override
        public void init() {
        }

        @Override
        public void loop() {
            Thread thread = new Thread(()->{
                double previousError = 0;
                double cumulativeError = 0;
                double dt = 0.1;
                while(true){
                    double currentProcessVariable = linearSlide.getCurrentPosition();
                    double error = desiredSetpoint - currentProcessVariable;
                    cumulativeError += error * dt;
                    double derivative = (error - previousError) / dt;
                    double controlOutput = K_p * error + K_i * cumulativeError + K_d * derivative;
                    controlOutput = Math.max(-1, Math.min(1, controlOutput));
                    linearSlide.setPower(controlOutput);
                    previousError = error;
                    try {
                        Thread.sleep((long)dt);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            });
            thread.start();

            telemetry.addData("pos ", linearSlide.getCurrentPosition());
            telemetry.addData("target pos" , desiredSetpoint);
            telemetry.update();
        }

        public void setTargetPosition(int pos){
            desiredSetpoint = pos;
        }
    }

