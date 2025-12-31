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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-48, -49, Math.toRadians(-125)))

                .strafeTo(new Vector2d(-23, -23))
                //pause to shoot then move it so that it's ready to intake a ball
                .strafeToLinearHeading(new Vector2d(-12, -30), Math.toRadians(90))
                .strafeTo(new Vector2d(-12, -33))
                //pause to rotate
                .strafeTo(new Vector2d(-12, -35))
                //pause to rotate
                .strafeTo(new Vector2d(-12, -39))
                //pause to rotate to shooting pos
                .strafeToLinearHeading(new Vector2d(-23, -23), Math.toRadians(-125))

                //.turn(Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}