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
    public static final Pose2d redWings_DecisionPointPos = new Pose2d(40,10, Math.toRadians(270));


    public static TrajectorySequence blueBackstage_ToDecisionPoint;
    public static TrajectorySequence blueWings_ToDecisionPoint , blueWings_ToMiddlePark;
    public static TrajectorySequence redBackstage_ToDecisionPoint , redBackstage_ToParkedPos;
    public static TrajectorySequence redWings_StartPositionToDecisionPoint, redWings_DecisionPointToMiddlePark, redWings_DecisionPointToCenterSpike, redWings_DecisionPointToLeftSpike, redWings_DecisionPointToRightSpike, redWings_LeftSpikeToDecisionPoint, redWings_RightSpikeToDecisionPoint, redWings_CenterSpikeToDecisionPoint ;

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
        redWings_StartPositionToDecisionPoint = //shift and go forward to score center
                drive.trajectorySequenceBuilder(redWings_StartPos)
                        .lineTo(new Vector2d(40, 64.5), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .lineTo(redWings_DecisionPointPos.vec(), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .build();

        redWings_DecisionPointToLeftSpike = //shift and go forward to score center
                drive.trajectorySequenceBuilder(redWings_DecisionPointPos)
                        .lineToLinearHeading(new Pose2d(40,26, Math.toRadians(225)))
                        .build();

        redWings_LeftSpikeToDecisionPoint =  //Shift to decision point
                drive.trajectorySequenceBuilder(redWings_DecisionPointToLeftSpike.end())
                        .lineTo(new Vector2d(42, 26), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .build();


        redWings_DecisionPointToCenterSpike = //shift and go forward to score center
                drive.trajectorySequenceBuilder(redWings_DecisionPointPos)
                        .lineToLinearHeading(new Pose2d(40,18, Math.toRadians(270)))
                        .build();

        redWings_CenterSpikeToDecisionPoint =  //Shift to decision point
                drive.trajectorySequenceBuilder(redWings_DecisionPointToCenterSpike.end())
                        .lineToLinearHeading(redWings_DecisionPointPos)
                        .build();

        redWings_DecisionPointToRightSpike = //shift and go forward to score center
                drive.trajectorySequenceBuilder(redWings_DecisionPointPos)
                        .lineToLinearHeading(new Pose2d(40,25, Math.toRadians(0)))
                        .build();

        redWings_RightSpikeToDecisionPoint =  //Shift to decision point
                drive.trajectorySequenceBuilder(redWings_DecisionPointToRightSpike.end())
                        .lineToLinearHeading(redWings_DecisionPointPos)
                        .build();


        redWings_DecisionPointToMiddlePark = //go to and spin to parking between backdrops
                drive.trajectorySequenceBuilder(redWings_StartPositionToDecisionPoint.end())
                        .lineToLinearHeading(new Pose2d( -42, 16, Math.toRadians(0)))
                        .build();


    }
}