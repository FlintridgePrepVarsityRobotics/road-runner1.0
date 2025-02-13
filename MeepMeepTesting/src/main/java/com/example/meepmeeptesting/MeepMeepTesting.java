package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

       // myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
//                .splineToConstantHeading(new Vector2d(20, 0), Math.PI / 2)
//                .splineToConstantHeading(new Vector2d(27, 0), Math.PI / 2)
//                        .waitSeconds(3)
//                .splineToConstantHeading(new Vector2d(27.6, -29), 0)
//                .splineToConstantHeading(new Vector2d(56, -29), 0)
//                .splineToConstantHeading(new Vector2d(56, -36), 0)
//                .splineToConstantHeading(new Vector2d(15, -36), 0)
//                .splineToConstantHeading(new Vector2d(56, -36), 0)
//                .splineToConstantHeading(new Vector2d(56, -46), 0)
//                .splineToConstantHeading(new Vector2d(15, -46), 0)
//                .splineToConstantHeading(new Vector2d(56, -46), 0)
//                .splineToConstantHeading(new Vector2d(56, -56), 0)
//                .splineToConstantHeading(new Vector2d(15, -56), 0)
//                .splineToConstantHeading(new Vector2d(32, -30), Math.PI / 2)
//                .splineToConstantHeading(new Vector2d(5, -30), 0)
//                        .waitSeconds(3)
//                .splineToConstantHeading(new Vector2d(27.6, 0), 0)
//                        .waitSeconds(3)
//                .splineToConstantHeading(new Vector2d(15, -40), 0)
//                .build());
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -60, Math.PI/2))
                .waitSeconds(3)
                .splineToConstantHeading(new Vector2d(10, -33), Math.PI / 2)
                //.splineToConstantHeading(new Vector2d(27, -33), Math.PI / 2)
                .waitSeconds(3)
                .splineToConstantHeading(new Vector2d(39, -33), 0)
                .splineToConstantHeading(new Vector2d(39, -10), 0)
                        .waitSeconds(3)
                .splineToConstantHeading(new Vector2d(49, -10), 0)
                .splineToConstantHeading(new Vector2d(49, -55), 0)
                .splineToConstantHeading(new Vector2d(56, -36), 0)
                .splineToConstantHeading(new Vector2d(56, -46), 0)
                .splineToConstantHeading(new Vector2d(15, -46), 0)
                .splineToConstantHeading(new Vector2d(56, -46), 0)
                .splineToConstantHeading(new Vector2d(56, -56), 0)
                .splineToConstantHeading(new Vector2d(15, -56), 0)
                .splineToConstantHeading(new Vector2d(32, -30), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(5, -30), 0)
                .waitSeconds(3)
                .splineToConstantHeading(new Vector2d(27.6, 0), 0)
                .waitSeconds(3)
                .splineToConstantHeading(new Vector2d(15, -40), 0)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}