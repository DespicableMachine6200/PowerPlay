package org.Team6200;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class LinearSlidePID {
    // integral gain
    public float Ki = 1;
    //proportional gain
    public float Kp = 1;
    // derivative gain
    public float Kd = 1;

    //slide constants
    int slideMax = 3000;
    int slideMin = 571;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    SampleMecanumDrive robot;
    int targetPosition;
    DcMotor linearSlide;
    int threshold = 10;
    public void setKi(float val){
        Ki = val;
    }
    public void setKp(float val){
        Kp = val;
    }
    public void setKd(float val){
        Kd = val;
    }
    public void setTargetPosition(int pos){
        targetPosition = pos;
    }

    public LinearSlidePID(Telemetry telem, HardwareMap hwm, SampleMecanumDrive rbt){
        telemetry = telem;
        hardwareMap = hwm;
        robot = rbt;
        linearSlide = rbt.lmotor;
        startLoop();
    }
    void startLoop(){
        Runnable thread = new Thread(()->{
            while (true){
                int currentPosititon = linearSlide.getCurrentPosition();
                int comparison = compare(currentPosititon, targetPosition);
                if(comparison != 1){
                    if(comparison == 2){
                        int difference = targetPosition - currentPosititon;
                        //proportional tuning
                        double power = processDifferenceToPower(difference);
                        linearSlide.setPower(power);
                    }
                    if(comparison == 3){
                        int difference = currentPosititon - targetPosition;
                        double power = processDifferenceToPower(difference);
                        linearSlide.setPower(0-power);
                    }
                }else{
                    linearSlide.setPower(0);
                }
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
        thread.run();
    }
    int compare(int actual, int target){
        if(differenceBetweenThreshold(actual, target, threshold)){
            //within threshold
            return 1;
        }else{
            if(actual < target){
                //needs to go up
                return 2;
            }else{
                // needs to go down
                return 3;
            }
        }
    }

    boolean differenceBetweenThreshold(int num1, int num2, int thres){
        if(num1 > num2){
            if(num1 - num2 < thres){
                return true;
            }else{
                return false;
            }
        }else if(num2 > num1){
            if(num2 - num1 < thres){
                return true;
            }else{
                return false;
            }
        }else{
            return true;
        }
    }
    double processDifferenceToPower(int difference){
        //proportional
        double power;
        float minPower = 0.5F;
        float maxPower = 1F;
        float slowThreshold = 0.8F;
        int slideRange = slideMax - slideMin;
        if(difference > slideRange*(1-slowThreshold)){
            power = 1;
        }else{
            // not necessarily porportional but I feel this
            //is better in this case
            power = 0.4;
        }
        return power;
    }
}
