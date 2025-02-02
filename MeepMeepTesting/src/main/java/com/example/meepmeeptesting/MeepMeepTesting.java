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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .splineToConstantHeading(new Vector2d(20, 0), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(27, 0), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(27, -26), 0)
                .splineToConstantHeading(new Vector2d(60, -26), 0)
                .splineToConstantHeading(new Vector2d(60, -36), 0)
                .splineToConstantHeading(new Vector2d(15, -36), 0)
                .splineToConstantHeading(new Vector2d(60, -36), 0)
                .splineToConstantHeading(new Vector2d(60, -46), 0)
                .splineToConstantHeading(new Vector2d(15, -46), 0)
                .splineToConstantHeading(new Vector2d(60, -46), 0)
                .splineToConstantHeading(new Vector2d(60, -56), 0)
                .splineToConstantHeading(new Vector2d(15, -56), 0)
                .splineToConstantHeading(new Vector2d(32, -30), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(5, -30), 0)
                .splineToConstantHeading(new Vector2d(20, 0), 0)
                .splineToConstantHeading(new Vector2d(15, -40), Math.PI / 2)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}