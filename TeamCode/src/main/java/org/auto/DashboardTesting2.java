package org.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class DashboardTesting2 {
        Telemetry telemetry;
        HardwareMap hardwareMap;
        SampleMecanumDrive robot;
        DcMotor linearSlide;
        public DashboardTesting2(Telemetry telem, HardwareMap hwm, SampleMecanumDrive rbt){
            telemetry = telem;
            hardwareMap = hwm;
            robot = rbt;
            linearSlide = rbt.lmotor;
            startLoop();
        }
        public double desiredSetpoint;
        void startLoop(){
            Thread thread = new Thread(()->{
                double K_p = 0.5;
                double K_i = 0.01;
                double K_d = 0.1;
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

            telemetry.addData("current pos" , linearSlide.getCurrentPosition());
            telemetry.update();
        }

        public void setTargetPosition(int pos){
            desiredSetpoint = pos;
        }
    }

