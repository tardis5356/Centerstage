package org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_ParkAutos;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.ARTEMIS.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.TESTBED.auto.CSTB_DriveConstants;
//import org.firstinspires.ftc.teamcode.TESTBED.auto.CSTB_SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ARTEMIS.trajectorysequence.TrajectorySequence;

public class Artemis_ParkAutoTrajectories {


    //BLUE BACKSTAGE
    public static final Pose2d blueBackstage_StartPos = new Pose2d(-8.5, -64.5, Math.toRadians(90));
    public static final Pose2d blueBackstage_CornerPark = new Pose2d(-52, -64.5, Math.toRadians(90));

    //BLUE WINGS
    public static final Pose2d blueWings_StartPos = new Pose2d(36.5, -64.5, Math.toRadians(90));
    public static final Pose2d blueWings_DecisionPointPos = new Pose2d(40.5, -15, Math.toRadians(90));
    public static final Pose2d blueWings_BackIntoParkPos = new Pose2d(40.5, -15, Math.toRadians(0));

    //RED BACKSTAGE
    public static final Pose2d redBackstage_StartPos = new Pose2d(-8.5, 64.5, Math.toRadians(270));
    public static final Pose2d redBackstage_CornerPark = new Pose2d(-52, 64.5, Math.toRadians(270));

    //RED WINGS
    public static final Pose2d redWings_StartPos = new Pose2d(36.5, 64.5, Math.toRadians(270));
    public static final Pose2d redWings_DecisionPointPos = new Pose2d(40.5, 16, Math.toRadians(270));
    public static final Pose2d redWings_BackIntoParkPos = new Pose2d(40.5, 16, Math.toRadians(0));

    public static TrajectorySequence blueBackstage_StartPositionToCornerPark ;
    public static TrajectorySequence blueWings_StartPositionToDecisionPoint , blueWings_DecisionPointToCenterPark;
    public static TrajectorySequence redBackstage_StartPositionToCornerPark ;
    public static TrajectorySequence redWings_StartPositionToDecisionPoint , redWings_DecisionPointToCenterPark;

    public static void generateTrajectories(SampleMecanumDrive drive) {

        // 🟦🟦🟦 BLUE AUTO COMMANDS 🟦🟦🟦
        //backstage
        blueBackstage_StartPositionToCornerPark = //shift and go forward to score center
                drive.trajectorySequenceBuilder(blueBackstage_StartPos)
                        .lineTo(new Vector2d(-12.5, -64.5), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .lineTo(blueBackstage_CornerPark.vec(), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();

        //wings
        blueWings_StartPositionToDecisionPoint = //shift and go forward to score center
                drive.trajectorySequenceBuilder(blueWings_StartPos)
                        .lineTo(new Vector2d(40.5, -64.5), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .lineToLinearHeading(new Pose2d(40.51, -15, Math.toRadians(0)))
                        .lineTo(blueWings_DecisionPointPos.vec(), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
        blueWings_DecisionPointToCenterPark = //shift and go forward to score center
                drive.trajectorySequenceBuilder(blueWings_BackIntoParkPos)
                        .lineToLinearHeading(new Pose2d(-48, -17, Math.toRadians(0)))
                        .build();

        // 🟥🟥🟥 red auto commands 🟥🟥🟥
        //backstage
        redBackstage_StartPositionToCornerPark = //shift and go forward to score center
                drive.trajectorySequenceBuilder(redBackstage_StartPos)
                        .lineTo(new Vector2d(-12.5, 64.5), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .lineTo(redBackstage_CornerPark.vec(), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();

        //wings
        redWings_StartPositionToDecisionPoint = //shift and go forward to score center
                drive.trajectorySequenceBuilder(redWings_StartPos)
                        .lineTo(new Vector2d(40.5, 64.5), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .lineToLinearHeading(new Pose2d(40.51, 16, Math.toRadians(0)))
                        .lineTo(redWings_DecisionPointPos.vec(), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
        redWings_DecisionPointToCenterPark = //shift and go forward to score center
                drive.trajectorySequenceBuilder(redWings_BackIntoParkPos)
                        .lineToLinearHeading(new Pose2d(-48, 18, Math.toRadians(0)))
                        .build();
}}
