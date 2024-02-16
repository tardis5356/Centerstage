package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static final Pose2d redWings_StartPos = new Pose2d(36.5, 64.5, Math.toRadians(270));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(400);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep).setDimensions(16.5, 16)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width

                .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(
                        drive ->
                                //startToLeftSpike
                                drive.trajectorySequenceBuilder(redWings_StartPos)
                                        // left spike
//                                        .splineToLinearHeading(new Pose2d(35, 32, Math.toRadians(10)), 180) // left spike
//                                        .waitSeconds(0.5)
//                                        .splineToSplineHeading(new Pose2d(60, 30, 0), 0)

                                        // center spike
                                        .splineToLinearHeading(new Pose2d(43, 27, Math.toRadians(10)), 180) // center spike
                                        .waitSeconds(0.5)
                                        .splineToSplineHeading(new Pose2d(60, 30, 0), 0)

                                        // right spike
//                                        .splineToSplineHeading(new Pose2d(60, 30, 0), 0)

                                        // grab from stack
                                        .strafeLeft(4)
                                        .back(4)
                                        .waitSeconds(0.5)

                                        // general move from stack through door
                                        .lineToLinearHeading(new Pose2d(56, 12, Math.toRadians(0)))
                                        .lineToLinearHeading(new Pose2d(-48, 12, Math.toRadians(0)))

                                        // general move from stack through truss
//                                        .lineToLinearHeading(new Pose2d(56, 59, Math.toRadians(0)))
//                                        .lineToLinearHeading(new Pose2d(-48, 59, Math.toRadians(0)))

                                        .build()
                        /*drive ->
                        drive.trajectorySequenceBuilder(redBackstage_StartPos)
                                .splineTo(blueBackstage_DecisionPointPos.vec(), 90)
//                                .lineTo(blueBackstage_DecisionPointPos.vec())
                                .lineToLinearHeading(new Pose2d(-12.5, -30, Math.toRadians(180)))
                                .lineToLinearHeading(blueBackstage_DecisionPointPos)
                                .lineToLinearHeading(new Pose2d(-42, -60.5, Math.toRadians(0)))
                                .build()*/
                );

        Image img = null;
//        try { img = ImageIO.read(new File("/Users/sabinamiller/StudioProjects/Centerstage/MeepMeepTesting/src/main/java/com/example/meepmeeptesting/field-2023-juice-dark-flipped.png")); }
        try {
            img = ImageIO.read(new File("/Users/thesimg/Robotics/Centerstage/MeepMeepTesting/src/main/java/com/example/meepmeeptesting/field-2023-juice-dark-flipped.png"));
//            catch (IOException e) {}
//        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
            meepMeep.setBackground(img)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}