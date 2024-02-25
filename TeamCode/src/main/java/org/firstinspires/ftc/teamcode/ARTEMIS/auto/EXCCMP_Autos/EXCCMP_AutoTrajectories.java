package org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.ARTEMIS.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ARTEMIS.trajectorysequence.TrajectorySequence;

public class EXCCMP_AutoTrajectories {

    // blue cycle emojis \uD83D\uDFE6
    // red cycle emojis \uD83D\uDFE5

    public static final TrajectoryVelocityConstraint
            velConstraint70in = SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
            velConstraint55in = SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
            velConstraint40in = SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
            velConstraint20in = SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
            velConstraint10in = SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    public static final TrajectoryAccelerationConstraint
            accelConstraint25in = SampleMecanumDrive.getAccelerationConstraint(25),
            accelConstraint40in = SampleMecanumDrive.getAccelerationConstraint(40),
            accelConstraint70in = SampleMecanumDrive.getAccelerationConstraint(70);


    //BLUE BACKSTAGE
    public static final Pose2d blueBackstage_StartPos = new Pose2d(-8, -62, Math.toRadians(90));

    //BLUE WINGS
    public static final Pose2d blueWings_StartPos = new Pose2d(32, -62, Math.toRadians(90));

    //RED BACKSTAGE
    public static final Pose2d redBackstage_StartPos = new Pose2d(-8, 62, Math.toRadians(270));

    //RED WINGS
    public static final Pose2d redWings_StartPos = new Pose2d(32, 62, Math.toRadians(270));

    // RED
    public static final Pose2d Red_InnerStackPos = new Pose2d(56, 12, Math.toRadians(0)); //originally x=58
    public static final Pose2d Red_CenterStackPos = new Pose2d(56, 24, Math.toRadians(0));
    public static final Pose2d Red_OuterStackPos = new Pose2d(56, 36, Math.toRadians(0));

    public static final Pose2d Red_DoorStackTransitWaypoint = new Pose2d(56, 12, Math.toRadians(0));
    public static final Pose2d Red_TrussStackTransitWaypoint = new Pose2d(50, 12, Math.toRadians(0));

    public static final Pose2d Red_DoorBackdropTransitWaypoint = new Pose2d(-40, 22, Math.toRadians(-40));
    public static final Pose2d Red_TrussBackdropTransitWaypoint = new Pose2d(-40, 49, Math.toRadians(40));

    public static final Pose2d Red_BackdropLeftPos = new Pose2d(-49, 29.5, Math.toRadians(0));
    public static final Pose2d Red_BackdropCenterPos = new Pose2d(-49, 35.5, Math.toRadians(0));
    public static final Pose2d Red_BackdropRightPos = new Pose2d(-49, 41.5, Math.toRadians(0));

    public static final Pose2d Red_CenterParkPos = new Pose2d(-50, 12, Math.toRadians(0));
    public static final Pose2d Red_CornerParkPos = new Pose2d(-50, 50, Math.toRadians(0));

    public static TrajectorySequence RedWings_StartToOrigin, RedBackstage_StartToOrigin, BlueWings_StartToOrigin, BlueBackstage_StartToOrigin;

    public static TrajectorySequence RedWings_StartToLeftSpike, RedWings_StartToCenterSpike, RedWings_StartToRightSpike;
    public static TrajectorySequence RedWings_LeftSpikeToStack, RedWings_CenterSpikeToStack, RedWings_RightSpikeToStack;
    public static TrajectorySequence RedWings_CenterStackToDoorWaypoint, RedWings_InnerStackToDoorWaypoint, RedWings_OuterStackToDoorWaypoint;
    public static TrajectorySequence RedWings_TransitToBackstageViaDoor, RedWings_TransitToBackstageViaTruss, RedWings_DoorStackTransitWaypointToBackdropWaypointViaDoor, RedWings_TransitToBackdropViaTruss;
    public static TrajectorySequence RedWings_TransitToBackstageViaDoorWait, RedWings_TransitToBackstageViaTrussWait, RedWings_TransitToBackdropViaDoorWait, RedWings_TransitToBackdropViaTrussWait;

    public static TrajectorySequence RedBackstage_StartToLeftSpike, RedBackstage_StartToCenterSpike, RedBackstage_StartToRightSpike;
    public static TrajectorySequence RedBackstage_SpikeToBackdrop;

    public static TrajectorySequence Red_BackdropToStackViaTruss, Red_BackdropToStackViaDoor, Red_BackstageToStackViaTruss, Red_BackstageToStackViaDoor;
    public static TrajectorySequence Red_DoorBackdropTransitWaypointToBackdropRight, Red_DoorBackdropTransitWaypointToBackdropCenter, Red_DoorBackdropTransitWaypointToBackdropLeft;
    public static TrajectorySequence Red_TrussBackdropTransitWaypointToBackdropRight, Red_TrussBackdropTransitWaypointToBackdropCenter, Red_TrussBackdropTransitWaypointToBackdropLeft;
    public static TrajectorySequence Red_BackdropLeftToBackdropCenter, Red_BackdropLeftToBackdropRight, Red_BackdropCenterToBackdropLeft, Red_BackdropCenterToBackdropRight, Red_BackdropRightToBackdropCenter, Red_BackdropRightToBackdropLeft;

    public static TrajectorySequence Red_BackdropLeftToCenterPark, Red_BackdropLeftToCornerPark, Red_BackdropCenterToCenterPark, Red_BackdropCenterToCornerPark, Red_BackdropRightToCenterPark, Red_BackdropRightToCornerPark;

    // backdrop to stack variants
    public static TrajectorySequence Red_BackdropLeftToStackViaTruss, Red_BackdropCenterToStackViaTruss, Red_BackdropRightToStackViaTruss;
    public static TrajectorySequence Red_BackdropLeftToStackViaDoor, Red_BackdropCenterToStackViaDoor, Red_BackdropRightToStackViaDoor;


    public static void generateTrajectories(SampleMecanumDrive drive) {
        // redwings 🟥🟥🟥🟥
        // red wings 🟥🪽 start 🏁 to spikes 🌵
        RedWings_StartToLeftSpike =
                drive.trajectorySequenceBuilder(redWings_StartPos)
                        .splineToLinearHeading(new Pose2d(37, 32, Math.toRadians(10)), 180, velConstraint70in, accelConstraint40in) // left spike
                        .build();
        RedWings_StartToCenterSpike =
                drive.trajectorySequenceBuilder(redWings_StartPos)
                        .splineToLinearHeading(new Pose2d(43, 26, Math.toRadians(10)), 180, velConstraint70in, accelConstraint40in) // center spike
                        .build();
        RedWings_StartToRightSpike =
                drive.trajectorySequenceBuilder(redWings_StartPos)
                        .splineToSplineHeading(new Pose2d(58, 30, 0), 0, velConstraint70in, accelConstraint40in)
                        .build();

        // redwings 🟥🪽 spike 🌵 to stack 🥞
        RedWings_LeftSpikeToStack =
                drive.trajectorySequenceBuilder(RedWings_StartToLeftSpike.end())
                        .splineToSplineHeading(Red_CenterStackPos, 0, velConstraint55in, accelConstraint40in)
                        .build();
        RedWings_CenterSpikeToStack =
                drive.trajectorySequenceBuilder(RedWings_StartToCenterSpike.end())
                        .splineToSplineHeading(Red_CenterStackPos, 0, velConstraint55in, accelConstraint40in)
                        .build();
        RedWings_RightSpikeToStack = drive.trajectorySequenceBuilder(RedWings_StartToRightSpike.end())
                .lineTo(Red_CenterStackPos.vec())
                .build();

        // redWings 🟥🪽 stack pickup 🥞🥞
        RedWings_CenterStackToDoorWaypoint = drive.trajectorySequenceBuilder(Red_CenterStackPos)
                .back(2, velConstraint20in, accelConstraint40in)
                .lineTo(Red_DoorStackTransitWaypoint.vec())
                .build();
        RedWings_OuterStackToDoorWaypoint = drive.trajectorySequenceBuilder(Red_OuterStackPos)
                .back(2, velConstraint20in, accelConstraint40in)
                .lineTo(Red_DoorStackTransitWaypoint.vec())
                .build();
        RedWings_InnerStackToDoorWaypoint = drive.trajectorySequenceBuilder(Red_InnerStackPos)
                .back(2, velConstraint20in, accelConstraint40in)
                .back(0.01)
                .lineTo(Red_DoorStackTransitWaypoint.vec())
                .back(0.01)
                .build();

        // redWings 🟥🪽 transit to back 🎭
        RedWings_DoorStackTransitWaypointToBackdropWaypointViaDoor = drive.trajectorySequenceBuilder(Red_DoorStackTransitWaypoint)
                .lineToLinearHeading(new Pose2d(-18, 11, Math.toRadians(0)), velConstraint70in, accelConstraint70in)
                .splineToSplineHeading(Red_DoorBackdropTransitWaypoint, Math.toRadians(140), velConstraint70in, accelConstraint40in)
                .build();//-49 x

        // red door stack transit waypoint to backdrop transit waypoint
        RedWings_TransitToBackdropViaDoorWait = drive.trajectorySequenceBuilder(Red_DoorStackTransitWaypoint)
//                .lineToLinearHeading(new Pose2d(56, 12, Math.toRadians(0)))
                .waitSeconds(10)
                .lineToLinearHeading(new Pose2d(-18, 12, Math.toRadians(0)))
                .splineToSplineHeading(Red_DoorBackdropTransitWaypoint, Math.toRadians(140))
                .build();

        RedWings_TransitToBackdropViaTruss = drive.trajectorySequenceBuilder(RedWings_CenterStackToDoorWaypoint.end())
                .lineToLinearHeading(new Pose2d(56, 59, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-18, 59, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-40, 49, Math.toRadians(40)), Math.toRadians(220))
                .build();
        RedWings_TransitToBackdropViaTrussWait = drive.trajectorySequenceBuilder(RedWings_CenterStackToDoorWaypoint.end())
                .lineToLinearHeading(new Pose2d(56, 59, Math.toRadians(0)))
                .waitSeconds(10)
                .lineToLinearHeading(new Pose2d(-18, 59, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-40, 49, Math.toRadians(40)), Math.toRadians(220))
                .build();

        RedWings_TransitToBackstageViaDoor = drive.trajectorySequenceBuilder(RedWings_CenterStackToDoorWaypoint.end())
//                .lineToLinearHeading(new Pose2d(56, 12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-48, 12, Math.toRadians(0)))
                .build();
        RedWings_TransitToBackstageViaDoorWait = drive.trajectorySequenceBuilder(RedWings_CenterStackToDoorWaypoint.end())
//                .lineToLinearHeading(new Pose2d(56, 12, Math.toRadians(0)))
                .waitSeconds(10)
                .lineToLinearHeading(new Pose2d(-48, 12, Math.toRadians(0)))
                .build();

        RedWings_TransitToBackstageViaTruss = drive.trajectorySequenceBuilder(RedWings_CenterStackToDoorWaypoint.end())
                .lineToLinearHeading(new Pose2d(56, 59, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-48, 59, Math.toRadians(0)))
                .build();

        RedWings_TransitToBackstageViaTrussWait = drive.trajectorySequenceBuilder(RedWings_CenterStackToDoorWaypoint.end())
                .lineToLinearHeading(new Pose2d(56, 59, Math.toRadians(0)))
                .waitSeconds(10)
                .lineToLinearHeading(new Pose2d(-48, 59, Math.toRadians(0)))
                .build();

        Red_DoorBackdropTransitWaypointToBackdropCenter = drive.trajectorySequenceBuilder(Red_DoorBackdropTransitWaypoint)
                .lineToLinearHeading(Red_BackdropCenterPos, velConstraint40in, accelConstraint25in)
                .build();
        Red_DoorBackdropTransitWaypointToBackdropLeft = drive.trajectorySequenceBuilder(Red_DoorBackdropTransitWaypoint)
                .lineToLinearHeading(Red_BackdropLeftPos, velConstraint40in, accelConstraint25in)
                .build();
        Red_DoorBackdropTransitWaypointToBackdropRight = drive.trajectorySequenceBuilder(Red_DoorBackdropTransitWaypoint)
                .lineToLinearHeading(Red_BackdropRightPos, velConstraint40in, accelConstraint25in)
                .build();

        Red_TrussBackdropTransitWaypointToBackdropCenter = drive.trajectorySequenceBuilder(Red_TrussBackdropTransitWaypoint)
                .lineToLinearHeading(Red_BackdropCenterPos, velConstraint40in, accelConstraint25in)
                .build();
        Red_TrussBackdropTransitWaypointToBackdropLeft = drive.trajectorySequenceBuilder(Red_TrussBackdropTransitWaypoint)
                .lineToLinearHeading(Red_BackdropLeftPos, velConstraint40in, accelConstraint25in)
                .build();
        Red_TrussBackdropTransitWaypointToBackdropRight = drive.trajectorySequenceBuilder(Red_TrussBackdropTransitWaypoint)
                .lineToLinearHeading(Red_BackdropRightPos, velConstraint40in, accelConstraint25in)
                .build();

        // backdrop to backdrop
        Red_BackdropLeftToBackdropCenter = drive.trajectorySequenceBuilder(Red_BackdropLeftPos)
                .lineToLinearHeading(Red_BackdropCenterPos, velConstraint40in, accelConstraint25in)
                .build();
        Red_BackdropLeftToBackdropRight = drive.trajectorySequenceBuilder(Red_BackdropLeftPos)
                .lineToLinearHeading(Red_BackdropRightPos, velConstraint40in, accelConstraint25in)
                .build();
        Red_BackdropCenterToBackdropLeft = drive.trajectorySequenceBuilder(Red_BackdropCenterPos)
                .lineToLinearHeading(Red_BackdropLeftPos, velConstraint40in, accelConstraint25in)
                .build();
        Red_BackdropCenterToBackdropRight = drive.trajectorySequenceBuilder(Red_BackdropCenterPos)
                .lineToLinearHeading(Red_BackdropRightPos, velConstraint40in, accelConstraint25in)
                .build();
        Red_BackdropRightToBackdropCenter = drive.trajectorySequenceBuilder(Red_BackdropRightPos)
                .lineToLinearHeading(Red_BackdropCenterPos, velConstraint40in, accelConstraint25in)
                .build();
        Red_BackdropRightToBackdropLeft = drive.trajectorySequenceBuilder(Red_BackdropRightPos)
                .lineToLinearHeading(Red_BackdropLeftPos, velConstraint40in, accelConstraint25in)
                .build();

        // RED BACKSTAGE 🟥🟥🟥🟥
        // redBackstage 🟥🎭 start 🏁 to spike 🌵
        RedBackstage_StartToRightSpike = drive.trajectorySequenceBuilder(redBackstage_StartPos)
                .splineToLinearHeading(new Pose2d(-12, 30, Math.toRadians(180)), Math.toRadians(290), velConstraint70in, accelConstraint40in)
                .build();
        RedBackstage_StartToCenterSpike = drive.trajectorySequenceBuilder(redBackstage_StartPos)
                .splineToLinearHeading(new Pose2d(-25, 27, Math.toRadians(160)), 180, velConstraint70in, accelConstraint40in) // center spike
                .build();
        RedBackstage_StartToLeftSpike = drive.trajectorySequenceBuilder(redBackstage_StartPos)
                .splineToSplineHeading(new Pose2d(-35, 34, Math.toRadians(180)), Math.toRadians(180), velConstraint70in, accelConstraint40in) // left spike
                .build();

        // redBackstage 🟥🎭 spike 🌵 to backdrop 🎭 //TODO: fix
//        RedBackstage_SpikeToBackdropLeft = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .lineToLinearHeading(new Pose2d(-40, 34, Math.toRadians(0)))
//                .build();
//        RedBackstage_SpikeToBackdropCenter = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .lineToLinearHeading(new Pose2d(-40, 34, Math.toRadians(0)))
//                .build();
//        RedBackstage_SpikeToBackdropRight = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .lineToLinearHeading(new Pose2d(-40, 34, Math.toRadians(0)))
//                .build();

        // RED PARK
        Red_BackdropLeftToCenterPark = drive.trajectorySequenceBuilder(Red_BackdropLeftPos)
                .lineTo(new Pose2d(Red_CenterParkPos.getX() - 18, Red_CenterParkPos.getY(), Math.toRadians(0)).vec())
                .lineTo(Red_CenterParkPos.vec())
                .build();
        Red_BackdropLeftToCornerPark = drive.trajectorySequenceBuilder(Red_BackdropLeftPos)
                .lineTo(new Pose2d(Red_CornerParkPos.getX() - 18, Red_CornerParkPos.getY(), Math.toRadians(0)).vec())
                .lineTo(Red_CornerParkPos.vec())
                .build();
        Red_BackdropCenterToCenterPark = drive.trajectorySequenceBuilder(Red_BackdropCenterPos)
                .lineTo(new Pose2d(Red_CenterParkPos.getX() - 18, Red_CenterParkPos.getY(), Math.toRadians(0)).vec())
                .lineTo(Red_CenterParkPos.vec())
                .build();
        Red_BackdropCenterToCornerPark = drive.trajectorySequenceBuilder(Red_BackdropCenterPos)
                .lineTo(new Pose2d(Red_CornerParkPos.getX() - 18, Red_CornerParkPos.getY(), Math.toRadians(0)).vec())
                .lineTo(Red_CornerParkPos.vec())
                .build();
        Red_BackdropRightToCenterPark = drive.trajectorySequenceBuilder(Red_BackdropRightPos)
                .lineTo(new Pose2d(Red_CenterParkPos.getX() - 18, Red_CenterParkPos.getY(), Math.toRadians(0)).vec())
                .lineTo(Red_CenterParkPos.vec())
                .build();
        Red_BackdropRightToCornerPark = drive.trajectorySequenceBuilder(Red_BackdropRightPos)
                .lineTo(new Pose2d(Red_CornerParkPos.getX() - 18, Red_CornerParkPos.getY(), Math.toRadians(0)).vec())
                .lineTo(Red_CornerParkPos.vec())
                .build();

        // RED CYCLE 🟥🔁
        Red_BackdropToStackViaTruss = drive.trajectorySequenceBuilder(Red_DoorBackdropTransitWaypointToBackdropCenter.end())
                .lineToLinearHeading(new Pose2d(56, 59, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-18, 59, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-40, 49, Math.toRadians(40)), Math.toRadians(220))
                .build();
        Red_BackdropToStackViaDoor = drive.trajectorySequenceBuilder(Red_DoorBackdropTransitWaypointToBackdropCenter.end())
                .lineToLinearHeading(new Pose2d(56, 12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-18, 12, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-40, 22, Math.toRadians(-40)), Math.toRadians(140))
                .build();

        Red_BackstageToStackViaTruss = drive.trajectorySequenceBuilder(RedWings_CenterStackToDoorWaypoint.end())
                .lineToLinearHeading(new Pose2d(56, 59, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-48, 59, Math.toRadians(0)))
                .build();
        Red_BackstageToStackViaDoor = drive.trajectorySequenceBuilder(RedWings_CenterStackToDoorWaypoint.end())
//                .lineToLinearHeading(new Pose2d(56, 12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-48, 12, Math.toRadians(0)))
                .build();


        /// TEST TRAJECTORIES
        RedWings_StartToOrigin = drive.trajectorySequenceBuilder(redWings_StartPos)
                .strafeLeft(5)
                .forward(50)
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();
        RedBackstage_StartToOrigin = drive.trajectorySequenceBuilder(redBackstage_StartPos)
                .strafeRight(5)
                .forward(50)
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();
        BlueWings_StartToOrigin = drive.trajectorySequenceBuilder(blueWings_StartPos)
                .strafeRight(5)
                .forward(50)
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();
        BlueBackstage_StartToOrigin = drive.trajectorySequenceBuilder(blueBackstage_StartPos)
                .strafeLeft(5)
                .forward(50)
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();
    }
}
