package org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class CSTB_AutoTrajectories {
    //    public static final Pose2d blue_StartPos = new Pose2d(-30.5, 64, Math.toRadians(90)); 2023-03-04

    //BLUE BACKSTAGE
    public static final Pose2d blueBackstage_StartPos = new Pose2d(-8, -4.5, Math.toRadians(90));
    public static final Pose2d blueBackstage_WaypointParkPos = new Pose2d(-61.5, 13.5, Math.toRadians(180));
    public static final Pose2d blueBackstage_ParkedPos = new Pose2d(-63, 63, Math.toRadians(220));

   //BLUE WINGS
    public static final Pose2d blueWings_StartPos = new Pose2d(32, -64.5, Math.toRadians(0));
    public static final Pose2d blueWings_WaypointParkPos = new Pose2d(-61.5, 13.5, Math.toRadians(180));
    public static final Pose2d blueWings_ParkedPos = new Pose2d(-63, 63, Math.toRadians(180));

   //RED BACKSTAGE
    public static final Pose2d redBackstage_StartPos = new Pose2d(-8, 64.5, Math.toRadians(90));
    public static final Pose2d redBackstage_WaypointParkPos = new Pose2d(-61.5, 13.5, Math.toRadians(180));
    public static final Pose2d redBackstage_ParkedPos = new Pose2d(-63, -63, Math.toRadians(180));

    //RED WINGS
    public static final Pose2d redWings_StartPos = new Pose2d(32, 64.5, Math.toRadians(40));
    public static final Pose2d redWings_WaypointParkPos = new Pose2d(-61.5, 13.5, Math.toRadians(180));
    public static final Pose2d redWings_ParkedPos = new Pose2d(-63, -63, Math.toRadians(180));


    public static TrajectorySequence blueBackstage_StartToBackstage, blueWings_StartToBackstage;
    public static TrajectorySequence redBackstage_StartToBackstage, redWings_StartToBackstage;

    public static void generateTrajectories(CSTB_SampleMecanumDrive drive) {
        // 🟦🟦🟦 BLUE AUTO COMMANDS 🟦🟦🟦
        blueBackstage_StartToBackstage =
                drive.trajectorySequenceBuilder(blueBackstage_StartPos)
                        .lineTo(new Vector2d(38, 62), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .setReversed(false)
                        .build();

        blueWings_StartToBackstage =
                drive.trajectorySequenceBuilder(blueWings_StartPos)
                        .setReversed(true)
                        .lineTo(new Vector2d(32, 0), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .setReversed(false)
                        .build();


        // 🟥🟥🟥 red auto commands 🟥🟥🟥
        redBackstage_StartToBackstage =
                drive.trajectorySequenceBuilder(redBackstage_StartPos)
                        .setReversed(true)
                        .lineTo(new Vector2d(-38, 62), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .setReversed(false)
                        .build();

        redWings_StartToBackstage =
                drive.trajectorySequenceBuilder(redWings_StartPos)
                        .setReversed(true)
                        .lineTo(new Vector2d(32, 24), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .setReversed(false)
                        .build();

    }
}