package org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleParkAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.ARTEMIS.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ARTEMIS.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;

public class Artemis_PurpleParkAutoTrajectories {

    //BLUE BACKSTAGE
    public static final Pose2d blueBackstage_StartPos = new Pose2d(-8.5, -64.5, Math.toRadians(90));
    public static final Pose2d blueBackstage_DecisionPointPos = new Pose2d(-12.5, -34, Math.toRadians(90));
    public static final Pose2d blueBackstage_XXXXSpikeMarkPos = new Pose2d(-14.5, -31, Math.toRadians(90));

    //RED BACKSTAGE
    public static final Pose2d redBackstage_StartPos = new Pose2d(-8.5, -64.5, Math.toRadians(270));
    public static final Pose2d redBackstage_DecisionPointPos = new Pose2d(-12.5, -34, Math.toRadians(270));
    public static final Pose2d redBackstage_XXXXSpikeMarkPos = new Pose2d(-14.5, -31, Math.toRadians(270));

    public static TrajectorySequence blueBackstage_StartPositionToDecisionPoint, blueBackstage_DecisionPointToCenterSpike, blueBackstage_DecisionPointToLeftSpike, blueBackstage_DecisionPointToRightSpike, blueBackstage_LeftSpikeToDecisionPoint, blueBackstage_RightSpikeToDecisionPoint, blueBackstage_CenterSpikeToDecisionPoint, blueBackstage_DecisionPointToCornerPark;
    public static TrajectorySequence redBackstage_StartPositionToDecisionPoint, redBackstage_DecisionPointToCenterSpike, redBackstage_DecisionPointToLeftSpike, redBackstage_DecisionPointToRightSpike, redBackstage_LeftSpikeToDecisionPoint, redBackstage_RightSpikeToDecisionPoint, redBackstage_CenterSpikeToDecisionPoint, redBackstage_DecisionPointToCornerPark;

    public static void generateTrajectories(SampleMecanumDrive drive) {

        //blue backstage
        {
            blueBackstage_StartPositionToDecisionPoint = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(blueBackstage_StartPos)
                            .lineTo(new Vector2d(-12.5, -64.5), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .lineTo(blueBackstage_DecisionPointPos.vec(), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();

            blueBackstage_DecisionPointToLeftSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(blueBackstage_DecisionPointPos)
                            .lineToLinearHeading(new Pose2d(-12.5, -30, Math.toRadians(180)))
                            .build();

            blueBackstage_LeftSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(blueBackstage_DecisionPointToLeftSpike.end())
                            .lineToLinearHeading(blueBackstage_DecisionPointPos)
                            .build();

            blueBackstage_DecisionPointToCenterSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(blueBackstage_DecisionPointPos)
                            .lineToLinearHeading(new Pose2d(-12.5, -30, Math.toRadians(90)))
                            .build();

            blueBackstage_CenterSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(blueBackstage_DecisionPointToCenterSpike.end())
                            .lineToLinearHeading(blueBackstage_DecisionPointPos)
                            .build();

            blueBackstage_DecisionPointToRightSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(blueBackstage_DecisionPointPos)
                            .lineToLinearHeading(new Pose2d(-12.5, -32, Math.toRadians(0)))
                            .build();

            blueBackstage_RightSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(blueBackstage_DecisionPointToRightSpike.end())
                            .lineToLinearHeading(blueBackstage_DecisionPointPos)
                            .build();


            blueBackstage_DecisionPointToCornerPark = //go to and spin to parking between backdrops
                    drive.trajectorySequenceBuilder(blueBackstage_StartPositionToDecisionPoint.end())
                            .lineToLinearHeading(new Pose2d(-52, -64.5, Math.toRadians(0)))
                            .build();
        }

        //red backstage
        {
            redBackstage_StartPositionToDecisionPoint = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(redBackstage_StartPos)
                            .lineTo(new Vector2d(-12.5, 64.5), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .lineTo(redBackstage_DecisionPointPos.vec(), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();

            redBackstage_DecisionPointToLeftSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(redBackstage_DecisionPointPos)
                            .lineToLinearHeading(new Pose2d(-12.5, 30, Math.toRadians(180)))
                            .build();

            redBackstage_LeftSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(redBackstage_DecisionPointToLeftSpike.end())
                            .lineToLinearHeading(redBackstage_DecisionPointPos)
                            .build();

            redBackstage_DecisionPointToCenterSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(redBackstage_DecisionPointPos)
                            .lineToLinearHeading(new Pose2d(-12.5, 30, Math.toRadians(90)))
                            .build();

            redBackstage_CenterSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(redBackstage_DecisionPointToCenterSpike.end())
                            .lineToLinearHeading(redBackstage_DecisionPointPos)
                            .build();

            redBackstage_DecisionPointToRightSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(redBackstage_DecisionPointPos)
                            .lineToLinearHeading(new Pose2d(-12.5, 32, Math.toRadians(0)))
                            .build();

            redBackstage_RightSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(redBackstage_DecisionPointToRightSpike.end())
                            .lineToLinearHeading(redBackstage_DecisionPointPos)
                            .build();


            redBackstage_DecisionPointToCornerPark = //go to and spin to parking between backdrops
                    drive.trajectorySequenceBuilder(redBackstage_StartPositionToDecisionPoint.end())
                            .lineToLinearHeading(new Pose2d(-52, 64.5, Math.toRadians(0)))
                            .build();
        }

    }
}
