package org.firstinspires.ftc.teamcode.TESTBED.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.TESTBED.trajectorysequence.TrajectorySequence;

public class CSTB_AutoTrajectories {

    //BLUE BACKSTAGE
    public static final Pose2d blueBackstage_StartPos = new Pose2d(-8, -64.5, Math.toRadians(90));
    public static final Pose2d blueBackstage_DecisionPointPos = new Pose2d(-12, -8, Math.toRadians(90));
    public static final Pose2d blueBackstage_PostScoreWaypoint = new Pose2d(-14, -18, Math.toRadians(90));
    //BLUE WINGS
    public static final Pose2d blueWings_StartPos = new Pose2d(36, -64.5, Math.toRadians(90));
    public static final Pose2d blueWings_DecisionPointPos = new Pose2d(40, -8, Math.toRadians(90));

    //RED BACKSTAGE
    public static final Pose2d redBackstage_StartPos = new Pose2d(-8, 64.5, Math.toRadians(270));
    public static final Pose2d redBackstage_DecisionPointPos = new Pose2d(-12, 8, Math.toRadians(270));
    public static final Pose2d redBackstage_PostScoreWaypoint = new Pose2d(-14, 18, Math.toRadians(270));

//    public static TrajectorySequence redBackstage_ToDecisionPoint, redBackstage_ToMiddlePark, redBackstage_ToCenterSpike, redBackstage_ToLeftSpike, redBackstage_ToRightSpike, redBackstage_ToParkedPos;
//    public static final Pose2d redBackstage_ToDecisionPoint = new Pose2d(-45,13, Math.toRadians(270));

    //RED WINGS
    public static final Pose2d redWings_StartPos = new Pose2d(36, 64.5, Math.toRadians(270));
    public static final Pose2d redWings_DecisionPointPos = new Pose2d(40,10, Math.toRadians(270));

    public static TrajectorySequence blueBackstage_StartPositionToDecisionPoint, blueBackstage_DecisionPointToMiddlePark, blueBackstage_DecisionPointToCenterSpike, blueBackstage_DecisionPointToLeftSpike, blueBackstage_DecisionPointToRightSpike, blueBackstage_LeftSpikeToDecisionPoint, blueBackstage_RightSpikeToDecisionPoint, blueBackstage_CenterSpikeToDecisionPoint ;
    public static TrajectorySequence blueWings_StartPositionToDecisionPoint, blueWings_DecisionPointToMiddlePark, blueWings_DecisionPointToCenterSpike, blueWings_DecisionPointToLeftSpike, blueWings_DecisionPointToRightSpike, blueWings_LeftSpikeToDecisionPoint, blueWings_RightSpikeToDecisionPoint, blueWings_CenterSpikeToDecisionPoint ;
    public static TrajectorySequence redBackstage_StartPositionToDecisionPoint, redBackstage_DecisionPointToMiddlePark, redBackstage_DecisionPointToCenterSpike, redBackstage_DecisionPointToLeftSpike, redBackstage_DecisionPointToRightSpike, redBackstage_LeftSpikeToDecisionPoint, redBackstage_RightSpikeToDecisionPoint, redBackstage_CenterSpikeToDecisionPoint ;
    public static TrajectorySequence redWings_StartPositionToDecisionPoint, redWings_DecisionPointToMiddlePark, redWings_DecisionPointToCenterSpike, redWings_DecisionPointToLeftSpike, redWings_DecisionPointToRightSpike, redWings_LeftSpikeToDecisionPoint, redWings_RightSpikeToDecisionPoint, redWings_CenterSpikeToDecisionPoint ;

    public static void generateTrajectories(CSTB_SampleMecanumDrive drive) {

        // 🟦🟦🟦 BLUE AUTO COMMANDS 🟦🟦🟦
        blueBackstage_StartPositionToDecisionPoint = //shift and go forward to score center
                drive.trajectorySequenceBuilder(blueBackstage_StartPos)
                        .lineTo(new Vector2d(-12, -64.5), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .lineTo(blueBackstage_DecisionPointPos.vec(), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .build();

        blueBackstage_DecisionPointToLeftSpike = //shift and go forward to score center
                drive.trajectorySequenceBuilder(blueBackstage_DecisionPointPos)
                        .lineToLinearHeading(new Pose2d(-12, -26, Math.toRadians(225)))
                        .build();

        blueBackstage_LeftSpikeToDecisionPoint =  //Shift to decision point
                drive.trajectorySequenceBuilder(blueBackstage_DecisionPointToLeftSpike.end())
                        .lineTo(new Vector2d(-12, -24), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .build();


        blueBackstage_DecisionPointToCenterSpike = //shift and go forward to score center
                drive.trajectorySequenceBuilder(blueBackstage_DecisionPointPos)
                        .lineToLinearHeading(new Pose2d(-12, -18, Math.toRadians(180)))
                        .build();

        blueBackstage_CenterSpikeToDecisionPoint =  //Shift to decision point
                drive.trajectorySequenceBuilder(blueBackstage_DecisionPointToCenterSpike.end())
                        .lineToLinearHeading(blueBackstage_DecisionPointPos)
                        .build();

        blueBackstage_DecisionPointToRightSpike = //shift and go forward to score center
                drive.trajectorySequenceBuilder(blueBackstage_DecisionPointPos)
                        .lineToLinearHeading(new Pose2d(-14, -18, Math.toRadians(315)))
                        .build();

        blueBackstage_RightSpikeToDecisionPoint =  //Shift to decision point
                drive.trajectorySequenceBuilder(blueBackstage_DecisionPointToRightSpike.end())
                        .lineToLinearHeading(blueBackstage_DecisionPointPos)
                        .build();


        blueBackstage_DecisionPointToMiddlePark = //go to and spin to parking between backdrops
                drive.trajectorySequenceBuilder(blueBackstage_PostScoreWaypoint)
                        .lineToLinearHeading(new Pose2d(-42, -60, Math.toRadians(0)))
                        .build();

        //blueWings
        {
            blueWings_StartPositionToDecisionPoint = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(blueWings_StartPos)
                            .lineTo(new Vector2d(40, -64.5), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                    CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                            .lineTo(blueWings_DecisionPointPos.vec(), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                    CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                            .build();

            blueWings_DecisionPointToLeftSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(blueWings_DecisionPointPos)
                            .lineToLinearHeading(new Pose2d(40, -26, Math.toRadians(45)))
                            .build();

            blueWings_LeftSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(blueWings_DecisionPointToLeftSpike.end())
                            .lineTo(new Vector2d(40, -24), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                    CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                            .build();


            blueWings_DecisionPointToCenterSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(blueWings_DecisionPointPos)
                            .lineToLinearHeading(new Pose2d(40, -18, Math.toRadians(90)))
                            .build();

            blueWings_CenterSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(blueWings_DecisionPointToCenterSpike.end())
                            .lineToLinearHeading(blueWings_DecisionPointPos)
                            .build();

            blueWings_DecisionPointToRightSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(blueWings_DecisionPointPos)
                            .lineToLinearHeading(new Pose2d(40, -20, Math.toRadians(135)))
                            .build();

            blueWings_RightSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(blueWings_DecisionPointToRightSpike.end())
                            .lineToLinearHeading(blueWings_DecisionPointPos)
                            .build();


            blueWings_DecisionPointToMiddlePark = //go to and spin to parking between backdrops
                    drive.trajectorySequenceBuilder(blueWings_StartPositionToDecisionPoint.end())
                            .lineToLinearHeading(new Pose2d(-42, -16, Math.toRadians(0)))
                            .build();
        }

        // 🟥🟥🟥 red auto commands 🟥🟥🟥

        {//redBackstage
        redBackstage_StartPositionToDecisionPoint = //shift and go forward to score center
                drive.trajectorySequenceBuilder(redBackstage_StartPos)
                        .lineTo(new Vector2d(-12, 64.5), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .lineTo(redBackstage_DecisionPointPos.vec(), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .build();

        redBackstage_DecisionPointToLeftSpike = //shift and go forward to score center
                drive.trajectorySequenceBuilder(redBackstage_DecisionPointPos)
                        .lineToLinearHeading(new Pose2d(-12, 26, Math.toRadians(225)))
                        .build();

        redBackstage_LeftSpikeToDecisionPoint =  //Shift to decision point
                drive.trajectorySequenceBuilder(redBackstage_DecisionPointToLeftSpike.end())
                        .lineTo(new Vector2d(-12, 24), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                        .build();


        redBackstage_DecisionPointToCenterSpike = //shift and go forward to score center
                drive.trajectorySequenceBuilder(redBackstage_DecisionPointPos)
                        .lineToLinearHeading(new Pose2d(-12, 18, Math.toRadians(270)))
                        .build();

        redBackstage_CenterSpikeToDecisionPoint =  //Shift to decision point
                drive.trajectorySequenceBuilder(redBackstage_DecisionPointToCenterSpike.end())
                        .lineToLinearHeading(redBackstage_DecisionPointPos)
                        .build();

        redBackstage_DecisionPointToRightSpike = //shift and go forward to score center
                drive.trajectorySequenceBuilder(redBackstage_DecisionPointPos)
                        .lineToLinearHeading(new Pose2d(-14, 18, Math.toRadians(135)))
                        .build();

        redBackstage_RightSpikeToDecisionPoint =  //Shift to decision point
                drive.trajectorySequenceBuilder(redBackstage_DecisionPointToRightSpike.end())
                        .lineToLinearHeading(redBackstage_DecisionPointPos)
                        .build();


        redBackstage_DecisionPointToMiddlePark = //go to and spin to parking between backdrops
                drive.trajectorySequenceBuilder(redBackstage_PostScoreWaypoint)
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
            redWings_StartPositionToDecisionPoint = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(redWings_StartPos)
                            .lineTo(new Vector2d(40, 64.5), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                    CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                            .lineTo(redWings_DecisionPointPos.vec(), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                    CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                            .build();

            redWings_DecisionPointToLeftSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(redWings_DecisionPointPos)
                            .lineToLinearHeading(new Pose2d(40, 26, Math.toRadians(225)))
                            .build();

            redWings_LeftSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(redWings_DecisionPointToLeftSpike.end())
                            .lineTo(new Vector2d(42, 26), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                                    CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                            .build();


            redWings_DecisionPointToCenterSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(redWings_DecisionPointPos)
                            .lineToLinearHeading(new Pose2d(40, 18, Math.toRadians(270)))
                            .build();

            redWings_CenterSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(redWings_DecisionPointToCenterSpike.end())
                            .lineToLinearHeading(redWings_DecisionPointPos)
                            .build();

            redWings_DecisionPointToRightSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(redWings_DecisionPointPos)
                            .lineToLinearHeading(new Pose2d(40, 25, Math.toRadians(0)))
                            .build();

            redWings_RightSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(redWings_DecisionPointToRightSpike.end())
                            .lineToLinearHeading(redWings_DecisionPointPos)
                            .build();


            redWings_DecisionPointToMiddlePark = //go to and spin to parking between backdrops
                    drive.trajectorySequenceBuilder(redWings_StartPositionToDecisionPoint.end())
                            .lineToLinearHeading(new Pose2d(-42, 16, Math.toRadians(0)))
                            .build();
        }
    }
}