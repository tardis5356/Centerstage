package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        final Pose2d blueBackstage_StartPos = new Pose2d(-8.5, -64.5, Math.toRadians(90));
        final Pose2d blueBackstage_DecisionPointPos = new Pose2d(-12.5, -34, Math.toRadians(90));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueBackstage_StartPos)
                                .lineTo(new Vector2d(-12.5, -64.5))
                                .lineTo(blueBackstage_DecisionPointPos.vec())
                                .lineToLinearHeading(new Pose2d(-12.5, -30, Math.toRadians(180)))
                                .lineToLinearHeading(blueBackstage_DecisionPointPos)
                                .lineToLinearHeading(new Pose2d(-42, -60.5, Math.toRadians(0)))
                                .build()
                );

        Image img = null;
        try { img = ImageIO.read(new File("/Users/sabinamiller/StudioProjects/Centerstage/MeepMeepTesting/src/main/java/com/example/meepmeeptesting/field-2023-juice-dark-flipped.png")); }
            catch (IOException e) {}
//        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}