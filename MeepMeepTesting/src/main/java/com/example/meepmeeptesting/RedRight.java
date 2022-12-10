package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

//meep meep example create your own class
public class RedRight {
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
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(0), 15)
                        .setDimensions(14, 10.5)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(35.5, -63, Math.toRadians(90)))
                                        .turn(Math.toRadians(90))
                                        .forward(24)
                                        .strafeRight(40)
                                        // drop cone
                                        .strafeRight(12)
                                        .turn(Math.toRadians(180))
                                        .forward(47)
                                        // pick up cone
                                        .back(35)
                                        .turn(Math.toRadians(90))
                                        // drop cone
                                        .turn(Math.toRadians(-90))
                                        .forward(35)
                                        // pick up cone
                                        .back(35)
                                        .turn(Math.toRadians(90))
                                        // drop cone
                                        .strafeRight(35)
                                        .forward(62)
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
                                drive.trajectorySequenceBuilder(new Pose2d(35.5, -63, Math.toRadians(90)))
                                        .turn(Math.toRadians(90))
                                        .forward(24)
                                        .strafeRight(40)
                                        // drop cone
                                        .strafeRight(12)
                                        .turn(Math.toRadians(180))
                                        .forward(47)
                                        // pick up cone
                                        .back(35)
                                        .turn(Math.toRadians(90))
                                        // drop cone
                                        .turn(Math.toRadians(-90))
                                        .forward(35)
                                        // pick up cone
                                        .back(35)
                                        .turn(Math.toRadians(90))
                                        // drop cone
                                        .strafeRight(12)
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
                                drive.trajectorySequenceBuilder(new Pose2d(35.5, -63, Math.toRadians(90)))
                                        .turn(Math.toRadians(90))
                                        .forward(24)
                                        .strafeRight(40)
                                        // drop cone
                                        .strafeRight(12)
                                        .turn(Math.toRadians(180))
                                        .forward(47)
                                        // pick up cone
                                        .back(35)
                                        .turn(Math.toRadians(90))
                                        // drop cone
                                        .turn(Math.toRadians(-90))
                                        .forward(35)
                                        // pick up cone
                                        .back(35)
                                        .turn(Math.toRadians(90))
                                        // drop cone
                                        .strafeRight(35)
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


