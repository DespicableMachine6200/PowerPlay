package org.Team6200;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class TeamPID {
    HardwareMap hardwareMap;
    SampleMecanumDrive robot;
    DcMotor backLeft = robot.leftRear;
    DcMotor backRight = robot.rightRear;
    DcMotor frontLeft = robot.leftFront;
    DcMotor frontRight = robot.rightFront;
    Telemetry telemetry;
    // possible tuning values idk if we gonna use them
    public float Kp = 1;
    public float Ki = 1;
    public float Kd = 1;
    int comparisonLastHalfSecondAverage = 0;
    int[] comparisons = new int[]{};
    float threshold = 0.1F;
    BNO055IMU imu = null;
    public double targetVelocity;
    double currentPower = 0;

    public TeamPID(HardwareMap hwm, SampleMecanumDrive rbt, Telemetry tele){
        hardwareMap = hwm;
        robot = rbt;
        telemetry = tele;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);
        startListening();
    }

    void startListening(){
        Runnable thread = new Thread(()->{
            int ticker = 0;

            while(true){
                telemetry.addLine(imu.getVelocity().xVeloc + "," + imu.getVelocity().yVeloc + ":" + targetVelocity);
                telemetry.addLine(imu.getPosition().toString());
                int comparison = compareVelocities(imu.getVelocity().xVeloc, imu.getVelocity().yVeloc, targetVelocity);
                comparisons[comparisons.length] = comparison;
                comparisonLastHalfSecondAverage = doAverage(comparisons);
                comparison = comparisonLastHalfSecondAverage;
                telemetry.addLine("Average: " + comparisonLastHalfSecondAverage);

                if(comparisonLastHalfSecondAverage != 1){
                    int multiplier = ticker/50;
                    if(comparison == 2){
                        currentPower += 0.01*multiplier;
                    }
                    if(comparison == 3){
                        currentPower -= 0.01*multiplier;
                    }
                    if(comparison == 5){
                        // correct rotation and individual motor speed
                    }
                    if(comparison == 6){
                        // correct rotation and individual motor speed
                    }
                }
                setPowerStraight(currentPower);
                telemetry.update();
                try {
                    Thread.sleep(10);
                    ticker += 10;
                    if(ticker >= 500){
                        ticker = 0;
                        comparisonLastHalfSecondAverage = 0;
                        comparisons = new int[]{};
                    }
                } catch (InterruptedException e){}
            }
        });
        thread.run();
    }

    void setPowerStraight(double power){
        backLeft.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(power);
    }

    int doAverage(int[] comparisons){
        int length = comparisons.length;
        int sum = 0;
        for(int i = 0; i < length; i++){
            sum += comparisons[i];
        }
        return (int)Math.floor((sum/length) + 1);
    }
    /*
    KEY
    1: on point
    2: too much
    3: too less
    4: on point on correct axis, but too much in opposing axis
    5: too much on correct axis, too much in opposing axis
    6: too less on correct axis, toom much in opposing axis
     */
    int compareVelocities(double actualX, double actualY, double target){
        double actual;
        double wrongActual;
        if(actualX > actualY){
            actual = actualX;
            wrongActual = actualY;
        }else{
            actual = actualY;
            wrongActual = actualX;
        }
        if(actual - target < threshold || target - actual < threshold){
            if(wrongActual - 0 < threshold || 0 - wrongActual < threshold){
                return 1;
            }else{
                return 4;
            }
        }else{
            if(actual > target){
                if(wrongActual - 0 < threshold || 0 - wrongActual < threshold){
                    return 2;
                }else{
                    return 5;
                }

            }else{
                if(wrongActual - 0 < threshold || 0 - wrongActual < threshold){
                    return 3;
                }else{
                    return 6;
                }
            }
        }
    }
    // possible tuning values idk if we gonna use them
    public void setKp(float val){
        Kp = val;
    }
    public void setKi(float val){
        Ki = val;
    }
    public void setKd(float val){
        Kd = val;
    }
    public void setTargetVelocity(double vel){
        targetVelocity = vel;
    }

    void updateC(){

    }
}
