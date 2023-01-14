package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

//meep meep example create your own class
public class BlueLeft {
    public static void main(String[] args) {
        //placeholder signal zone
        int signalZone = 1;
        MeepMeep meepMeep = new MeepMeep(400);
        RoadRunnerBotEntity myBot = null;

        switch (signalZone){
            case 1:
                System.out.println("Going to 1");
                myBot = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        // change maxAngVel to 270 cuts down 1.5 sec
                        .setDimensions(14, 10.5)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(35, 60, Math.toRadians(270)))
                                        .turn(Math.toRadians(-90))
                                        .forward(22)
                                        .strafeLeft(36)
                                        .strafeLeft(12)
                                        //.back(20)
                                        //.back(45)
                                        /*.strafeRight(20)
                                        .lineToLinearHeading(new Pose2d(10, 24, Math.toRadians(180)))
                                        // drop cone
                                        .lineToLinearHeading(new Pose2d(24, 13, Math.toRadians(-90)))
                                        .lineToLinearHeading(new Pose2d(60, 12, Math.toRadians(0)))
                                        // pick up cone
                                        .lineToLinearHeading(new Pose2d(24, 8, Math.toRadians(-90)))
                                        // drop cone
                                        .lineToLinearHeading(new Pose2d(60, 12, Math.toRadians(0)))
                                        // pick up cone
                                        .lineToLinearHeading(new Pose2d(24, 8, Math.toRadians(-90)))*/
                                        // drop cone, park
                                        //.lineToLinearHeading(new Pose2d(60, 13, Math.toRadians(-90)))
                                        //.lineToLinearHeading(new Pose2d(36, 13, Math.toRadians(-90)))
                                        //.lineToLinearHeading(new Pose2d(12, 13, Math.toRadians(-90)))
                                        /*.turn(Math.toRadians(-90))
                                        .forward(22)
                                        .strafeLeft(36)
                                        // drop cone
                                        .strafeLeft(12)
                                        .turn(Math.toRadians(180))
                                        .forward(50)
                                        // pick up cone
                                        .back(42)
                                        .turn(Math.toRadians(-90))
                                        // drop cone
                                        .turn(Math.toRadians(90))
                                        .forward(38)
                                        // pick up cone
                                        .back(38)
                                        .turn(Math.toRadians(-90))
                                        // drop cone
                                        .strafeLeft(38)*/
                                        .build()
                        );
                break;
            case 2:
                System.out.println("Going to 2");
                myBot = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .setDimensions(14, 10.5)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(37, 60, Math.toRadians(270)))
                                        .turn(Math.toRadians(-90))
                                        .forward(22)
                                        .strafeLeft(36)
                                        // drop cone
                                        .strafeLeft(12)
                                        .turn(Math.toRadians(180))
                                        .forward(50)
                                        // pick up cone
                                        .back(42)
                                        .turn(Math.toRadians(-90))
                                        // drop cone
                                        .turn(Math.toRadians(90))
                                        .forward(38)
                                        // pick up cone
                                        .back(38)
                                        .turn(Math.toRadians(-90))
                                        // drop cone
                                        .strafeLeft(20)
                                        .build()
                        );
                break;
            case 3:
                System.out.println("Going to 3");
                myBot = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .setDimensions(14, 10.5)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(37, 60, Math.toRadians(270)))
                                        .turn(Math.toRadians(-90))
                                        .forward(22)
                                        .strafeLeft(36)
                                        // drop cone
                                        .strafeLeft(12)
                                        .turn(Math.toRadians(180))
                                        .forward(50)
                                        // pick up cone
                                        .back(42)
                                        .turn(Math.toRadians(-90))
                                        // drop cone
                                        .turn(Math.toRadians(90))
                                        .forward(38)
                                        // pick up cone
                                        .back(38)
                                        .turn(Math.toRadians(-90))
                                        // drop cone
                                        .strafeRight(15)
                                        .build()
                        );

        }




        if(myBot != null){
            meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }

    }
}




