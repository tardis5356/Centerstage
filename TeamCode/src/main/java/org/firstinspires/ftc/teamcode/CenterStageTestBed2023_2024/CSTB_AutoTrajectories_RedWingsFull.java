package org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class CSTB_AutoTrajectories_RedWingsFull {

    //RED BACKSTAGE
    public static final Pose2d redBackstage_StartPos = new Pose2d(-8, 64.5, Math.toRadians(270));
    public static final Pose2d redBackstage_ParkedPos = new Pose2d(-61.5, 64.5, Math.toRadians(270));
    public static final Pose2d redBackstage_ToBackdrop = new Pose2d(-63, -63, Math.toRadians(270));

    //RED WINGS
    public static final Pose2d redWings_StartPos = new Pose2d(-36, 64.5, Math.toRadians(270));
    public static final Pose2d redWings_PostPurplePos = new Pose2d(40, 16, Math.toRadians(270));
    public static final Pose2d redWings_ParkedPos = new Pose2d(-63, -63, Math.toRadians(270));

    public static TrajectorySequence redWings_ToDecisionPoint, redWings_ToMiddlePark, redWings_ToCenterSpike, redWings_ToLeftSpike, redWings_ToRightSpike;
//    public static final Pose2d redWings_ToDecisionPoint = new Pose2d(-45,13, Math.toRadians(270));


    public static TrajectorySequence blueBackstage_ToDecisionPoint;
    public static TrajectorySequence blueWings_ToDecisionPoint, blueWings_ToMiddlePark;
    public static TrajectorySequence redBackstage_ToDecisionPoint, redBackstage_ToParkedPos;
//    public static void generateTrajectories() {
//        generateTrajectories(null);
//    }

    public static void generateTrajectories(CSTB_SampleMecanumDrive drive) {

//        class Scratch {
//            public void main(String[] args) {
        redBackstage_ToParkedPos =
                drive.trajectorySequenceBuilder(redBackstage_StartPos)
                        .setReversed(true)
                        .lineTo(new Vector2d(-36, 12), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .setReversed(false)
                        .build();


        //redWings
        redWings_ToDecisionPoint = //go forward to score center
                drive.trajectorySequenceBuilder(redWings_StartPos)
                        //.splineToConstantHeading(new Vector2d(-45,13), Math.toRadians(270))
                        .lineToLinearHeading(new Pose2d(-30, 10, Math.toRadians(270)))
                        .build();

        redWings_ToLeftSpike = //go forward to score center
                drive.trajectorySequenceBuilder(redWings_ToDecisionPoint.end())
                        .splineToConstantHeading(new Vector2d(-42, 22), Math.toRadians(270))
                        .build();

        redWings_ToCenterSpike = //go forward to score center
                drive.trajectorySequenceBuilder(redWings_ToDecisionPoint.end())
                        .splineToConstantHeading(new Vector2d(-36, 12.01), Math.toRadians(270))
                        .build();

        redWings_ToRightSpike = //go forward to score center
                drive.trajectorySequenceBuilder(redWings_ToDecisionPoint.end())
                        .splineToConstantHeading(new Vector2d(-38, 26), Math.toRadians(270))
                        .build();

        redWings_ToMiddlePark = //go to and spin to parking between backdrops
                drive.trajectorySequenceBuilder(redWings_ToDecisionPoint.end())
                        .splineToConstantHeading(new Vector2d(-42, 16), Math.toRadians(270))
                        .build();
    }
}
//    }
//}