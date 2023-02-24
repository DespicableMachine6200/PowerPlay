package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

//meep meep example create your own class
public class MeepMeepTesting {
    public static void main(String[] args) {
            MeepMeep meepMeep = new MeepMeep(800);

            RoadRunnerBotEntity robot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-35.5, -63, Math.toRadians(90)))
                                    // CONE 1 START
                                    .forward(60.9) //50.9
                                   // .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION))
                                   // .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.lmotor.setTargetPosition(3000))
                                   // .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.lmotor.setPower(1))
                                    .back(10)
                                    .strafeRight(13.5) //12.5
                                    .forward(6.6) //4.6
                                    .waitSeconds(1)//prob make 0.5 cuz it works for cone 2
                                    // drop cone 1
                                    //.addTemporalMarker(() -> robot.servo.setPosition(minPosition))
                                    .waitSeconds(0.5) //take out/shorten
                                    // CONE 1 END
                                    // CONE 2 START
                                    .back(11)
                                    .lineToLinearHeading(new Pose2d(-40, -11.5, Math.toRadians(180)))
                               //     .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> robot.servo.setPosition(maxPosition))
                                    .forward(27)
                                  //  .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(571))
                                   // .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                                  //  .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                                    .waitSeconds(0.5) //take out?
                                  //  .addTemporalMarker(() -> robot.lmotor.setTargetPosition(1000))
                                    // use claw
                                    .back(20)
                                    .lineToLinearHeading(new Pose2d(-26.2 , -10.6, Math.toRadians(90)))
                                    .forward(5.9) //4.6
                                 //   .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(3000))
                                    .waitSeconds(1.5)
                                    // drop cone 2
                               //     .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                                    .waitSeconds(0.5)
                                    // CONE 2 END
                                    // CONE 3 START
                                    .back(11)
                                    .lineToLinearHeading(new Pose2d(-40, -11.5, Math.toRadians(180)))
                                 //   .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> robot.servo.setPosition(maxPosition))
                                    .forward(27)
                                //    .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(400))
                                 //   .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                                //    .addTemporalMarker(() -> robot.servo.setPosition(maxPosition))
                                    .waitSeconds(0.5) //take out?
                                //    .addTemporalMarker(() -> robot.lmotor.setTargetPosition(1000))
                                    // use claw
                                    .back(20)
                                    .lineToLinearHeading(new Pose2d(-28.3 , -10.6, Math.toRadians(90)))
                                    .forward(4.6)
                               //     .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.lmotor.setTargetPosition(3000))
                                    .waitSeconds(1.5)
                                    // drop cone 3
                              //      .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.servo.setPosition(minPosition))
                                    .waitSeconds(0.5)
                                    // CONE 3 END
                                    // park
                                    .back(5.6)
                                    .strafeLeft(12)
                                    .build()
                    );

            meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(robot)
                    .start();
        }
    }


