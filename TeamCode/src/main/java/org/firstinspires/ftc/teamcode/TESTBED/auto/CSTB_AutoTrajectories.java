package org.firstinspires.ftc.teamcode.TESTBED.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.TESTBED.trajectorysequence.TrajectorySequence;

public class CSTB_AutoTrajectories {

    //BLUE BACKSTAGE
    public static final Pose2d CSTB_blueBackstage_StartPos = new Pose2d(-8, -64.5, Math.toRadians(90));
    public static final Pose2d CSTB_blueBackstage_DecisionPointPos = new Pose2d(-12, -8, Math.toRadians(90));
    public static final Pose2d CSTB_blueBackstage_PostScoreWaypoint = new Pose2d(-14, -18, Math.toRadians(90));
    //BLUE WINGS
    public static final Pose2d CSTB_blueWings_StartPos = new Pose2d(36, -64.5, Math.toRadians(90));
    public static final Pose2d CSTB_blueWings_DecisionPointPos = new Pose2d(40, -8, Math.toRadians(90));

    //RED BACKSTAGE
    public static final Pose2d CSTB_redBackstage_StartPos = new Pose2d(-8, 64.5, Math.toRadians(270));
    public static final Pose2d CSTB_redBackstage_DecisionPointPos = new Pose2d(-12, 8, Math.toRadians(270));
    public static final Pose2d CSTB_redBackstage_PostScoreWaypoint = new Pose2d(-14, 18, Math.toRadians(270));

//    public static TrajectorySequence redBackstage_ToDecisionPoint, redBackstage_ToMiddlePark, redBackstage_ToCenterSpike, redBackstage_ToLeftSpike, redBackstage_ToRightSpike, redBackstage_ToParkedPos;
//    public static final Pose2d redBackstage_ToDecisionPoint = new Pose2d(-45,13, Math.toRadians(270));

    //RED WINGS
    public static final Pose2d CSTB_redWings_StartPos = new Pose2d(36, 64.5, Math.toRadians(270));
    public static final Pose2d CSTB_redWings_DecisionPointPos = new Pose2d(40,10, Math.toRadians(270));

    public static TrajectorySequence CSTB_blueBackstage_StartPositionToDecisionPoint, CSTB_blueBackstage_DecisionPointToMiddlePark, CSTB_blueBackstage_DecisionPointToCenterSpike, CSTB_blueBackstage_DecisionPointToLeftSpike, CSTB_blueBackstage_DecisionPointToRightSpike, CSTB_blueBackstage_LeftSpikeToDecisionPoint, CSTB_blueBackstage_RightSpikeToDecisionPoint, CSTB_blueBackstage_CenterSpikeToDecisionPoint;
    public static TrajectorySequence CSTB_blueWings_StartPositionToDecisionPoint, CSTB_blueWings_DecisionPointToMiddlePark, CSTB_blueWings_DecisionPointToCenterSpike, CSTB_blueWings_DecisionPointToLeftSpike, CSTB_blueWings_DecisionPointToRightSpike, CSTB_blueWings_LeftSpikeToDecisionPoint, CSTB_blueWings_RightSpikeToDecisionPoint, CSTB_blueWings_CenterSpikeToDecisionPoint;
    public static TrajectorySequence CSTB_redBackstage_StartPositionToDecisionPoint, CSTB_redBackstage_DecisionPointToMiddlePark, CSTB_redBackstage_DecisionPointToCenterSpike, CSTB_redBackstage_DecisionPointToLeftSpike, CSTB_redBackstage_DecisionPointToRightSpike, CSTB_redBackstage_LeftSpikeToDecisionPoint, CSTB_redBackstage_RightSpikeToDecisionPoint, CSTB_redBackstage_CenterSpikeToDecisionPoint;
    public static TrajectorySequence CSTB_redWings_StartPositionToDecisionPoint, CSTB_redWings_DecisionPointToMiddlePark, CSTB_redWings_DecisionPointToCenterSpike, CSTB_redWings_DecisionPointToLeftSpike, CSTB_redWings_DecisionPointToRightSpike, CSTB_redWings_LeftSpikeToDecisionPoint, CSTB_redWings_RightSpikeToDecisionPoint, CSTB_redWings_CenterSpikeToDecisionPoint;

    public static void generateTrajectories(CSTB_SampleMecanumDrive drive) {

        // 🟦🟦🟦 BLUE AUTO COMMANDS 🟦🟦🟦
        CSTB_blueBackstage_StartPositionToDecisionPoint = //shift and go forward to score center
                drive.trajectorySequenceBuilder(CSTB_blueBackstage_StartPos)
                        .lineTo(new Vector2d(-12, -64.5), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .lineTo(CSTB_blueBackstage_DecisionPointPos.vec(), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .build();

        CSTB_blueBackstage_DecisionPointToLeftSpike = //shift and go forward to score center
                drive.trajectorySequenceBuilder(CSTB_blueBackstage_DecisionPointPos)
                        .lineToLinearHeading(new Pose2d(-12, -26, Math.toRadians(225)))
                        .build();

        CSTB_blueBackstage_LeftSpikeToDecisionPoint =  //Shift to decision point
                drive.trajectorySequenceBuilder(CSTB_blueBackstage_DecisionPointToLeftSpike.end())
                        .lineTo(new Vector2d(-12, -24), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .build();


        CSTB_blueBackstage_DecisionPointToCenterSpike = //shift and go forward to score center
                drive.trajectorySequenceBuilder(CSTB_blueBackstage_DecisionPointPos)
                        .lineToLinearHeading(new Pose2d(-12, -18, Math.toRadians(180)))
                        .build();

        CSTB_blueBackstage_CenterSpikeToDecisionPoint =  //Shift to decision point
                drive.trajectorySequenceBuilder(CSTB_blueBackstage_DecisionPointToCenterSpike.end())
                        .lineToLinearHeading(CSTB_blueBackstage_DecisionPointPos)
                        .build();

        CSTB_blueBackstage_DecisionPointToRightSpike = //shift and go forward to score center
                drive.trajectorySequenceBuilder(CSTB_blueBackstage_DecisionPointPos)
                        .lineToLinearHeading(new Pose2d(-14, -18, Math.toRadians(315)))
                        .build();

        CSTB_blueBackstage_RightSpikeToDecisionPoint =  //Shift to decision point
                drive.trajectorySequenceBuilder(CSTB_blueBackstage_DecisionPointToRightSpike.end())
                        .lineToLinearHeading(CSTB_blueBackstage_DecisionPointPos)
                        .build();


        CSTB_blueBackstage_DecisionPointToMiddlePark = //go to and spin to parking between backdrops
                drive.trajectorySequenceBuilder(CSTB_blueBackstage_PostScoreWaypoint)
                        .lineToLinearHeading(new Pose2d(-42, -60, Math.toRadians(0)))
                        .build();

        //blueWings
        {
            CSTB_blueWings_StartPositionToDecisionPoint = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(CSTB_blueWings_StartPos)
                            .lineTo(new Vector2d(40, -64.5), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                    CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                            .lineTo(CSTB_blueWings_DecisionPointPos.vec(), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                    CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                            .build();

            CSTB_blueWings_DecisionPointToLeftSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(CSTB_blueWings_DecisionPointPos)
                            .lineToLinearHeading(new Pose2d(40, -26, Math.toRadians(45)))
                            .build();

            CSTB_blueWings_LeftSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(CSTB_blueWings_DecisionPointToLeftSpike.end())
                            .lineTo(new Vector2d(40, -24), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                    CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                            .build();


            CSTB_blueWings_DecisionPointToCenterSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(CSTB_blueWings_DecisionPointPos)
                            .lineToLinearHeading(new Pose2d(40, -18, Math.toRadians(90)))
                            .build();

            CSTB_blueWings_CenterSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(CSTB_blueWings_DecisionPointToCenterSpike.end())
                            .lineToLinearHeading(CSTB_blueWings_DecisionPointPos)
                            .build();

            CSTB_blueWings_DecisionPointToRightSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(CSTB_blueWings_DecisionPointPos)
                            .lineToLinearHeading(new Pose2d(40, -20, Math.toRadians(135)))
                            .build();

            CSTB_blueWings_RightSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(CSTB_blueWings_DecisionPointToRightSpike.end())
                            .lineToLinearHeading(CSTB_blueWings_DecisionPointPos)
                            .build();


            CSTB_blueWings_DecisionPointToMiddlePark = //go to and spin to parking between backdrops
                    drive.trajectorySequenceBuilder(CSTB_blueWings_StartPositionToDecisionPoint.end())
                            .lineToLinearHeading(new Pose2d(-42, -16, Math.toRadians(0)))
                            .build();
        }

        // 🟥🟥🟥 red auto commands 🟥🟥🟥

        {//redBackstage
        CSTB_redBackstage_StartPositionToDecisionPoint = //shift and go forward to score center
                drive.trajectorySequenceBuilder(CSTB_redBackstage_StartPos)
                        .lineTo(new Vector2d(-12, 64.5), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .lineTo(CSTB_redBackstage_DecisionPointPos.vec(), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .build();

        CSTB_redBackstage_DecisionPointToLeftSpike = //shift and go forward to score center
                drive.trajectorySequenceBuilder(CSTB_redBackstage_DecisionPointPos)
                        .lineToLinearHeading(new Pose2d(-12, 26, Math.toRadians(225)))
                        .build();

        CSTB_redBackstage_LeftSpikeToDecisionPoint =  //Shift to decision point
                drive.trajectorySequenceBuilder(CSTB_redBackstage_DecisionPointToLeftSpike.end())
                        .lineTo(new Vector2d(-12, 24), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .build();


        CSTB_redBackstage_DecisionPointToCenterSpike = //shift and go forward to score center
                drive.trajectorySequenceBuilder(CSTB_redBackstage_DecisionPointPos)
                        .lineToLinearHeading(new Pose2d(-12, 18, Math.toRadians(270)))
                        .build();

        CSTB_redBackstage_CenterSpikeToDecisionPoint =  //Shift to decision point
                drive.trajectorySequenceBuilder(CSTB_redBackstage_DecisionPointToCenterSpike.end())
                        .lineToLinearHeading(CSTB_redBackstage_DecisionPointPos)
                        .build();

        CSTB_redBackstage_DecisionPointToRightSpike = //shift and go forward to score center
                drive.trajectorySequenceBuilder(CSTB_redBackstage_DecisionPointPos)
                        .lineToLinearHeading(new Pose2d(-14, 18, Math.toRadians(135)))
                        .build();

        CSTB_redBackstage_RightSpikeToDecisionPoint =  //Shift to decision point
                drive.trajectorySequenceBuilder(CSTB_redBackstage_DecisionPointToRightSpike.end())
                        .lineToLinearHeading(CSTB_redBackstage_DecisionPointPos)
                        .build();


        CSTB_redBackstage_DecisionPointToMiddlePark = //go to and spin to parking between backdrops
                drive.trajectorySequenceBuilder(CSTB_redBackstage_PostScoreWaypoint)
                        .lineToLinearHeading(new Pose2d(-42, 60, Math.toRadians(0)))
                        .build();}


        {//redBackstage EAVAN
//            redBackstage_ToDecisionPoint = //go forward to score center
//                    drive.trajectorySequenceBuilder(redBackstage_StartPos)
//                            //.splineToConstantHeading(new Vector2d(-45,13), Math.toRadians(270))
//                            .lineToLinearHeading(new Pose2d(-30, 10, Math.toRadians(270)))
//                            .build();
//
//            redBackstage_ToLeftSpike = //go forward to score center
//                    drive.trajectorySequenceBuilder(redBackstage_ToDecisionPoint.end())
//                            .forward(60.0002)
//                            .back(60.0001)
//                            .splineToConstantHeading(new Vector2d(-38, 50.1), Math.toRadians(270))
//                            .build();
//
//            redBackstage_ToCenterSpike = //go forward to score center
//                    drive.trajectorySequenceBuilder(redBackstage_ToDecisionPoint.end())
//                            .forward(50)
//                            .splineToConstantHeading(new Vector2d(-36, 12.01), Math.toRadians(270))
//                            .build();
//
//            redBackstage_ToRightSpike = //go forward to score center
//                    drive.trajectorySequenceBuilder(redBackstage_ToDecisionPoint.end())
//                            .forward(55)
//                            .splineToConstantHeading(new Vector2d(-35, 10.1), Math.toRadians(270))
//                            .build();
//
//            redBackstage_ToMiddlePark = //go to and spin to parking between backdrops
//                    drive.trajectorySequenceBuilder(redBackstage_ToDecisionPoint.end())
//                            .forward(36)
//                            .splineToConstantHeading(new Vector2d(-42, 20.01), Math.toRadians(270))
//                            .build();
        }



        //redWings
        {
            CSTB_redWings_StartPositionToDecisionPoint = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(CSTB_redWings_StartPos)
                            .lineTo(new Vector2d(40, 64.5), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                    CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                            .lineTo(CSTB_redWings_DecisionPointPos.vec(), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                    CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                            .build();

            CSTB_redWings_DecisionPointToLeftSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(CSTB_redWings_DecisionPointPos)
                            .lineToLinearHeading(new Pose2d(40, 26, Math.toRadians(225)))
                            .build();

            CSTB_redWings_LeftSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(CSTB_redWings_DecisionPointToLeftSpike.end())
                            .lineTo(new Vector2d(42, 26), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                    CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                            .build();


            CSTB_redWings_DecisionPointToCenterSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(CSTB_redWings_DecisionPointPos)
                            .lineToLinearHeading(new Pose2d(40, 18, Math.toRadians(270)))
                            .build();

            CSTB_redWings_CenterSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(CSTB_redWings_DecisionPointToCenterSpike.end())
                            .lineToLinearHeading(CSTB_redWings_DecisionPointPos)
                            .build();

            CSTB_redWings_DecisionPointToRightSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(CSTB_redWings_DecisionPointPos)
                            .lineToLinearHeading(new Pose2d(40, 25, Math.toRadians(0)))
                            .build();

            CSTB_redWings_RightSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(CSTB_redWings_DecisionPointToRightSpike.end())
                            .lineToLinearHeading(CSTB_redWings_DecisionPointPos)
                            .build();


            CSTB_redWings_DecisionPointToMiddlePark = //go to and spin to parking between backdrops
                    drive.trajectorySequenceBuilder(CSTB_redWings_StartPositionToDecisionPoint.end())
                            .lineToLinearHeading(new Pose2d(-42, 16, Math.toRadians(0)))
                            .build();
        }
    }
}