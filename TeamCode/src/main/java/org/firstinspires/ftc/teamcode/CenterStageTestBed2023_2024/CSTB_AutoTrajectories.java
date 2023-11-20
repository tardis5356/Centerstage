package org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class CSTB_AutoTrajectories {

    //BLUE BACKSTAGE
    public static final Pose2d blueBackstage_StartPos = new Pose2d(-8, 64.5, Math.toRadians(90));
    public static final Pose2d blueBackstage_WaypointParkPos = new Pose2d(-61.5, 13.5, Math.toRadians(180));
    public static final Pose2d blueBackstage_ParkedPos = new Pose2d(-63, 63, Math.toRadians(220));

   //BLUE WINGS
    public static final Pose2d blueWings_StartPos = new Pose2d(36, -64.5, Math.toRadians(90));
    public static final Pose2d blueWings_dropPurple = new Pose2d(-61.5, 13.5, Math.toRadians(270));
    public static final Pose2d blueWings_ParkedPos = new Pose2d(-63, 63, Math.toRadians(270));

   //RED BACKSTAGE
    public static final Pose2d redBackstage_StartPos = new Pose2d(-8, 64.5, Math.toRadians(270));
    public static final Pose2d redBackstage_ParkedPos = new Pose2d(-61.5, 64.5, Math.toRadians(270));
    public static final Pose2d redBackstage_ToBackdrop = new Pose2d(-63, -63, Math.toRadians(270));

    //RED WINGS
    public static final Pose2d redWings_StartPos = new Pose2d(36, 64.5, Math.toRadians(270));
    public static final Pose2d redWings_PostPurplePos = new Pose2d(40, 16, Math.toRadians(270));
    public static final Pose2d redWings_ParkedPos = new Pose2d(-63, -63, Math.toRadians(270));


    public static TrajectorySequence blueBackstage_ToDecisionPoint;
    public static TrajectorySequence blueWings_ToDecisionPoint , blueWings_ToMiddlePark;
    public static TrajectorySequence redBackstage_ToDecisionPoint , redBackstage_ToParkedPos;
    public static TrajectorySequence redWings_ToDecisionPoint , redWings_ToMiddlePark;

    public static void generateTrajectories(CSTB_SampleMecanumDrive drive) {

        // 🟦🟦🟦 BLUE AUTO COMMANDS 🟦🟦🟦
        blueBackstage_ToDecisionPoint =
                drive.trajectorySequenceBuilder(blueBackstage_StartPos)
                        .lineTo(new Vector2d(-40, 64.5), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .setReversed(false)
                        .build();

        //bluewings
        blueWings_ToDecisionPoint =
                drive.trajectorySequenceBuilder(blueWings_StartPos)
                        .lineTo(new Vector2d(40, -64.5), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .lineTo(new Vector2d(40, -8 ), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .build();

        blueWings_ToMiddlePark = //go to and spin to parking between backdrops
                drive.trajectorySequenceBuilder(blueWings_ToDecisionPoint.end())
                        .lineToLinearHeading(new Pose2d( -42, -16, Math.toRadians(0)))
                        //.lineTo(new Vector2d(-60, 10), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                        //CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .build();
        
        //blueWings_dropPurple =


        // 🟥🟥🟥 red auto commands 🟥🟥🟥
        redBackstage_ToParkedPos =
                drive.trajectorySequenceBuilder(redBackstage_StartPos)
                        .setReversed(true)
                        .lineTo(new Vector2d(-50, 64.5), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .setReversed(false)
                        .build();


        //redwings
        redWings_ToDecisionPoint = //shift and go forward to score center
                drive.trajectorySequenceBuilder(redWings_StartPos)
                        .lineTo(new Vector2d(40, 64.5), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .lineTo(new Vector2d(40, 12), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .build();

        redWings_ToMiddlePark = //go to and spin to parking between backdrops
                drive.trajectorySequenceBuilder(redWings_ToDecisionPoint.end())
                        .lineToLinearHeading(new Pose2d( -42, 16, Math.toRadians(180)))
                        //.lineTo(new Vector2d(-60, 10), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                //CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .build();


    }
}