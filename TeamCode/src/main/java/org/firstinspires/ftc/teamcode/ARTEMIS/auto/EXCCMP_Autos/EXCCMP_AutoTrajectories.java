package org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.ARTEMIS.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ARTEMIS.trajectorysequence.TrajectorySequence;

public class EXCCMP_AutoTrajectories {

    // blue cycle emojis \uD83D\uDFE6
    // red cycle emojis \uD83D\uDFE5

    public static final TrajectoryVelocityConstraint
            velConstraint70in = SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),//70
            velConstraint55in = SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
            velConstraint40in = SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
            velConstraint30in = SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
            velConstraint20in = SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
            velConstraint10in = SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    public static final TrajectoryAccelerationConstraint
            accelConstraint25in = SampleMecanumDrive.getAccelerationConstraint(25),
            accelConstraint40in = SampleMecanumDrive.getAccelerationConstraint(40),
            accelConstraint70in = SampleMecanumDrive.getAccelerationConstraint(55); //70


    //BLUE BACKSTAGE
    public static final Pose2d blueBackstage_StartPos = new Pose2d(-8, -62, Math.toRadians(90));

    //BLUE WINGS
    public static final Pose2d blueWings_StartPos = new Pose2d(32, -62, Math.toRadians(90));

    //RED BACKSTAGE
//    public static final Pose2d redBackstage_StartPos = new Pose2d(-8, 62, Math.toRadians(270));
    public static final Pose2d redBackstage_StartPos = new Pose2d(-8, 62, Math.toRadians(270));

    //RED WINGS
//    public static final Pose2d redWings_StartPos = new Pose2d(32, 62, Math.toRadians(270));
    public static final Pose2d redWings_StartPos = new Pose2d(32, 62, Math.toRadians(270));

    public static Pose2d startPose;

    // RED
    public static final Pose2d Red_InnerStackPos = new Pose2d(56, 12, Math.toRadians(0)); //originally x=58
    public static final Pose2d Red_CenterStackPos = new Pose2d(56, 24, Math.toRadians(0));
    public static final Pose2d Red_OuterStackPos = new Pose2d(56, 36, Math.toRadians(0));

    public static final Pose2d Red_DoorStackTransitWaypoint = new Pose2d(56, 12, Math.toRadians(0));
    public static final Pose2d Red_TrussStackTransitWaypoint = new Pose2d(56, 59, Math.toRadians(0));

    public static final Pose2d Red_DoorBackdropTransitWaypoint = new Pose2d(-40, 22, Math.toRadians(-40));
    public static final Pose2d Red_TrussBackdropTransitWaypoint = new Pose2d(-40, 48, Math.toRadians(20));

    public static final Pose2d Red_BackdropLeftPos = new Pose2d(-49, 30.5, Math.toRadians(0)); // x: 29.5
    public static final Pose2d Red_BackdropCenterPos = new Pose2d(-49, 36.5, Math.toRadians(0)); // x: 35.5
    public static final Pose2d Red_BackdropRightPos = new Pose2d(-49, 41.5, Math.toRadians(0)); // x: 41.5

    public static final Pose2d Red_BackdropLeftOffsetPos = new Pose2d(-46, 30.5, Math.toRadians(0)); // x: 29.5
    public static final Pose2d Red_BackdropCenterOffsetPos = new Pose2d(-46, 36.5, Math.toRadians(0)); // x: 35.5
    public static final Pose2d Red_BackdropRightOffsetPos = new Pose2d(-46, 41.5, Math.toRadians(0)); // x: 41.5

    public static final Pose2d Red_BackdropCenterRelocPos = new Pose2d(-35, 35.5, Math.toRadians(0)); // x: 35.5
    public static final Pose2d Red_BackdropLeftCenterRelocPos = new Pose2d(-35, 30, Math.toRadians(-20)); // x: 35.5
    public static final Pose2d Red_BackdropRightCenterRelocPos = new Pose2d(-35, 42, Math.toRadians(20)); // x: 35.5

    public static final Pose2d Red_CenterParkPos = new Pose2d(-58, 12, Math.toRadians(0));
    public static final Pose2d Red_CornerParkPos = new Pose2d(-58, 60, Math.toRadians(0));

    public static final Pose2d Red_CenterParkOffsetPos = new Pose2d(-50, 12, Math.toRadians(0));
    public static final Pose2d Red_CornerParkOffsetPos = new Pose2d(-50, 60, Math.toRadians(0));


    // test trajs
    public static TrajectorySequence RedWings_StartToOrigin, RedBackstage_StartToOrigin, BlueWings_StartToOrigin, BlueBackstage_StartToOrigin;

    // WINGS: start to spikes, spikes to stack
    public static TrajectorySequence RedWings_StartToLeftSpike, RedWings_StartToCenterSpike, RedWings_StartToRightSpike;
    public static TrajectorySequence RedWings_LeftSpikeToStack, RedWings_CenterSpikeToStack, RedWings_RightSpikeToStack;

    // BACKSTAGE: start to spikes, spikes to backdrop
    public static TrajectorySequence RedBackstage_StartToLeftSpike, RedBackstage_StartToCenterSpike, RedBackstage_StartToRightSpike;
    public static TrajectorySequence RedBackstage_LeftSpikeToBackdropWaypoint, RedBackstage_CenterSpikeToBackdropWaypoint, RedBackstage_RightSpikeToBackdropWaypoint;

    // stack to stack waypoints
    public static TrajectorySequence RedWings_CenterStackToDoorWaypoint, RedWings_InnerStackToDoorWaypoint, RedWings_OuterStackToDoorWaypoint;
    public static TrajectorySequence RedWings_CenterStackToTrussWaypoint, RedWings_InnerStackToTrussWaypoint, RedWings_OuterStackToTrussWaypoint;

    // transit
    public static TrajectorySequence RedWings_TransitToBackstageViaDoor, RedWings_TransitToBackstageViaTruss, RedWings_DoorStackWaypointToBackdropWaypointViaDoor, RedWings_TrussStackWaypointToBackdropWaypointViaTruss;
    public static TrajectorySequence RedWings_TransitToBackstageViaDoorWait, RedWings_TransitToBackstageViaTrussWait, Red_DoorStackWaypointToBackdropWaypointViaDoorWait, RedWings_TrussStackWaypointToBackdropWaypointViaTrussWait;

    public static TrajectorySequence RedBackstage_BackdropRelocWaypointToBackdropLeft, RedBackstage_BackdropRelocWaypointToBackdropCenter, RedBackstage_BackdropRelocWaypointToBackdropRight;

    public static TrajectorySequence Red_BackdropToStackViaTruss, Red_BackdropToStackViaDoor, Red_BackstageToStackViaTruss, Red_BackstageToStackViaDoor;
    public static TrajectorySequence Red_DoorBackdropTransitWaypointToBackdropRight, Red_DoorBackdropTransitWaypointToBackdropCenter, Red_DoorBackdropTransitWaypointToBackdropLeft;
    public static TrajectorySequence Red_TrussBackdropTransitWaypointToBackdropRight, Red_TrussBackdropTransitWaypointToBackdropCenter, Red_TrussBackdropTransitWaypointToBackdropLeft;
    public static TrajectorySequence Red_BackdropLeftToBackdropCenter, Red_BackdropLeftToBackdropRight, Red_BackdropCenterToBackdropLeft, Red_BackdropCenterToBackdropRight, Red_BackdropRightToBackdropCenter, Red_BackdropRightToBackdropLeft;

    public static TrajectorySequence Red_BackdropRightToBackdropWaypointDoor, Red_BackdropLeftToBackdropWaypointDoor, Red_BackdropCenterToBackdropWaypointDoor;
    public static TrajectorySequence Red_BackdropLeftToBackdropWaypointTruss, Red_BackdropRightToBackdropWaypointTruss, Red_BackdropCenterToBackdropWaypointTruss;

    public static TrajectorySequence Red_BackdropLeftToCenterPark, Red_BackdropLeftToCornerPark, Red_BackdropCenterToCenterPark, Red_BackdropCenterToCornerPark, Red_BackdropRightToCenterPark, Red_BackdropRightToCornerPark;

    public static TrajectorySequence DoorBackdropWaypointToStackWaypoint, TrussBackdropWaypointToStackWaypoint;

    // stack waypoint to stack
    public static TrajectorySequence DoorStackWaypointToInnerStack, DoorStackWaypointToCenterStack, DoorStackWaypointToOuterStack;
    public static TrajectorySequence TrussStackWaypointToInnerStack, TrussStackWaypointToCenterStack, TrussStackWaypointToOuterStack;

    // backdrop to stack variants
//    public static TrajectorySequence Red_BackdropLeftToStackViaTruss, Red_BackdropCenterToStackViaTruss, Red_BackdropRightToStackViaTruss;
//    public static TrajectorySequence Red_BackdropLeftToStackViaDoor, Red_BackdropCenterToStackViaDoor, Red_BackdropRightToStackViaDoor;


    public static void generateTrajectories(SampleMecanumDrive drive) {

        // set start position
        if (!SampleMecanumDrive.flipPose) { // red
            if (EXCCMP_Auto.getStartingSide() == "wing")
                startPose = redWings_StartPos;
            else
                startPose = redBackstage_StartPos;
        } else { // blue
            if (EXCCMP_Auto.getStartingSide() == "wing")
                startPose = blueWings_StartPos;
            else
                startPose = blueBackstage_StartPos;
        }

        // redwings 🟥🟥🟥🟥
        // red wings 🟥🪽 start 🏁 to spikes 🌵
        RedWings_StartToRightSpike =
                drive.trajectorySequenceBuilder(startPose)
                        .splineToLinearHeading(new Pose2d(38, 35, Math.toRadians(10)), 180, velConstraint70in, accelConstraint40in) // left spike
                        .build();
        RedWings_StartToCenterSpike =
                drive.trajectorySequenceBuilder(startPose)
                        .splineToLinearHeading(new Pose2d(52, 25, Math.toRadians(10)), 180, velConstraint70in, accelConstraint40in) // center spike
                        .build();
        RedWings_StartToLeftSpike =
                drive.trajectorySequenceBuilder(startPose)
                        .splineToSplineHeading(new Pose2d(50, 25, Math.toRadians(300)), Math.toRadians(300), velConstraint70in, accelConstraint40in)
                        .lineToLinearHeading(new Pose2d(50, 21, Math.toRadians(300)), velConstraint70in, accelConstraint40in)
                        .build();

        // redwings 🟥🪽 spike 🌵 to stack 🥞
        RedWings_LeftSpikeToStack =
                drive.trajectorySequenceBuilder(RedWings_StartToLeftSpike.end())
                        .lineToLinearHeading(Red_CenterStackPos, velConstraint40in, accelConstraint40in)
                        .build();
        RedWings_CenterSpikeToStack =
                drive.trajectorySequenceBuilder(RedWings_StartToCenterSpike.end())
                        .lineToLinearHeading(Red_CenterStackPos, velConstraint40in, accelConstraint40in)
                        .build();
        RedWings_RightSpikeToStack = drive.trajectorySequenceBuilder(RedWings_StartToRightSpike.end())
                .lineToLinearHeading(Red_CenterStackPos, velConstraint40in, accelConstraint40in)
                .build();

        // redWings 🟥🪽 stack pickup 🥞🥞
        RedWings_CenterStackToDoorWaypoint = drive.trajectorySequenceBuilder(RedWings_CenterSpikeToStack.end())
                .back(2, velConstraint20in, accelConstraint40in)
                .lineTo(Red_DoorStackTransitWaypoint.vec())
                .build();
        RedWings_OuterStackToDoorWaypoint = drive.trajectorySequenceBuilder(Red_OuterStackPos) /**no fixed start pos**/
                .back(2, velConstraint20in, accelConstraint40in)
                .lineTo(Red_DoorStackTransitWaypoint.vec())
                .build();
        RedWings_InnerStackToDoorWaypoint = drive.trajectorySequenceBuilder(Red_InnerStackPos) /**no fixed start pos**/
                .back(2, velConstraint20in, accelConstraint40in)
                .back(0.01)
                .lineTo(Red_DoorStackTransitWaypoint.vec())
                .build();

        RedWings_CenterStackToTrussWaypoint = drive.trajectorySequenceBuilder(RedWings_CenterSpikeToStack.end())
                .back(2, velConstraint20in, accelConstraint40in)
                .lineTo(Red_TrussStackTransitWaypoint.vec())
                .build();
        RedWings_OuterStackToTrussWaypoint = drive.trajectorySequenceBuilder(Red_OuterStackPos) /**no fixed start pos**/
                .back(2, velConstraint20in, accelConstraint40in)
                .lineTo(Red_TrussStackTransitWaypoint.vec())
                .build();
        RedWings_InnerStackToTrussWaypoint = drive.trajectorySequenceBuilder(Red_InnerStackPos) /**no fixed start pos**/
                .back(2, velConstraint20in, accelConstraint40in)
                .back(0.01)
                .lineTo(Red_TrussStackTransitWaypoint.vec())
                .build();


        // redWings 🟥🪽 transit to back 🎭
        RedWings_DoorStackWaypointToBackdropWaypointViaDoor = drive.trajectorySequenceBuilder(RedWings_CenterStackToDoorWaypoint.end())
                .lineToLinearHeading(new Pose2d(-18, 11, Math.toRadians(0)), velConstraint70in, accelConstraint70in)
                .splineToSplineHeading(Red_DoorBackdropTransitWaypoint, Math.toRadians(140), velConstraint70in, accelConstraint40in)
                .build();//-49 x
        // red door stack transit waypoint to backdrop transit waypoint
        Red_DoorStackWaypointToBackdropWaypointViaDoorWait = drive.trajectorySequenceBuilder(RedWings_CenterStackToDoorWaypoint.end())
                .waitSeconds(10)
                .lineToLinearHeading(new Pose2d(-18, 11, Math.toRadians(0)))
                .splineToSplineHeading(Red_DoorBackdropTransitWaypoint, Math.toRadians(140))
                .build();

        RedWings_TrussStackWaypointToBackdropWaypointViaTruss = drive.trajectorySequenceBuilder(RedWings_CenterStackToTrussWaypoint.end())
                .lineToLinearHeading(new Pose2d(-18, 59, Math.toRadians(0)), velConstraint70in, accelConstraint40in)
                .splineToSplineHeading(Red_TrussBackdropTransitWaypoint, Math.toRadians(220))
                .build();
        RedWings_TrussStackWaypointToBackdropWaypointViaTrussWait = drive.trajectorySequenceBuilder(RedWings_CenterStackToTrussWaypoint.end())
                .waitSeconds(10)
                .lineToLinearHeading(new Pose2d(-18, 59, Math.toRadians(0)))
                .splineToSplineHeading(Red_TrussBackdropTransitWaypoint, Math.toRadians(220))
                .build();

        Red_DoorBackdropTransitWaypointToBackdropCenter = drive.trajectorySequenceBuilder(RedWings_DoorStackWaypointToBackdropWaypointViaDoor.end())
                .lineToLinearHeading(Red_BackdropCenterOffsetPos, velConstraint40in, accelConstraint70in)
                .lineToLinearHeading(Red_BackdropCenterPos, velConstraint20in, accelConstraint25in)
                .build();
        Red_DoorBackdropTransitWaypointToBackdropLeft = drive.trajectorySequenceBuilder(RedWings_DoorStackWaypointToBackdropWaypointViaDoor.end())
                .lineToLinearHeading(Red_BackdropLeftOffsetPos, velConstraint40in, accelConstraint70in)
                .lineToLinearHeading(Red_BackdropLeftPos, velConstraint20in, accelConstraint25in)
                .build();
        Red_DoorBackdropTransitWaypointToBackdropRight = drive.trajectorySequenceBuilder(RedWings_DoorStackWaypointToBackdropWaypointViaDoor.end())
                .lineToLinearHeading(Red_BackdropRightOffsetPos, velConstraint40in, accelConstraint70in)
                .lineToLinearHeading(Red_BackdropRightPos, velConstraint20in, accelConstraint25in)
                .build();

        Red_TrussBackdropTransitWaypointToBackdropCenter = drive.trajectorySequenceBuilder(RedWings_TrussStackWaypointToBackdropWaypointViaTruss.end())
                .lineToLinearHeading(Red_BackdropCenterOffsetPos, velConstraint40in, accelConstraint70in)
                .lineToLinearHeading(Red_BackdropCenterPos, velConstraint20in, accelConstraint25in)
                .build();
        Red_TrussBackdropTransitWaypointToBackdropLeft = drive.trajectorySequenceBuilder(RedWings_TrussStackWaypointToBackdropWaypointViaTruss.end())
                .lineToLinearHeading(Red_BackdropLeftOffsetPos, velConstraint40in, accelConstraint70in)
                .lineToLinearHeading(Red_BackdropLeftPos, velConstraint20in, accelConstraint25in)
                .build();
        Red_TrussBackdropTransitWaypointToBackdropRight = drive.trajectorySequenceBuilder(RedWings_TrussStackWaypointToBackdropWaypointViaTruss.end())
                .lineToLinearHeading(Red_BackdropRightOffsetPos, velConstraint40in, accelConstraint70in)
                .lineToLinearHeading(Red_BackdropRightPos, velConstraint20in, accelConstraint25in)
                .build();

        // RED BACKSTAGE 🟥🟥🟥🟥
        // redBackstage 🟥🎭 start 🏁 to spike 🌵
        RedBackstage_StartToLeftSpike = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-14, 35, Math.toRadians(180)), Math.toRadians(290), velConstraint70in, accelConstraint40in)
                .build();
        RedBackstage_StartToCenterSpike = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-21, 28, Math.toRadians(160)), 180, velConstraint70in, accelConstraint40in) // center spike
                .build();
        RedBackstage_StartToRightSpike = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-35, 34, Math.toRadians(180)), Math.toRadians(180), velConstraint70in, accelConstraint40in) // left spike
                .build();

        // redBackstage 🟥🎭 spike 🌵 to backdrop 🎭
        RedBackstage_LeftSpikeToBackdropWaypoint = drive.trajectorySequenceBuilder(RedBackstage_StartToLeftSpike.end())
                .lineToLinearHeading(Red_BackdropLeftCenterRelocPos, velConstraint40in, accelConstraint70in)
                .build();
        RedBackstage_BackdropRelocWaypointToBackdropLeft = drive.trajectorySequenceBuilder(RedBackstage_LeftSpikeToBackdropWaypoint.end())
                .lineToLinearHeading(Red_BackdropLeftOffsetPos, velConstraint40in, accelConstraint70in)
                .lineToLinearHeading(Red_BackdropLeftPos, velConstraint20in, accelConstraint25in)
                .build();

        RedBackstage_CenterSpikeToBackdropWaypoint = drive.trajectorySequenceBuilder(RedBackstage_StartToCenterSpike.end())
                .lineToLinearHeading(Red_BackdropLeftCenterRelocPos, velConstraint40in, accelConstraint70in)
                .build();
        RedBackstage_BackdropRelocWaypointToBackdropCenter = drive.trajectorySequenceBuilder(RedBackstage_CenterSpikeToBackdropWaypoint.end())
                .lineToLinearHeading(Red_BackdropCenterOffsetPos, velConstraint40in, accelConstraint70in)
                .lineToLinearHeading(Red_BackdropCenterPos, velConstraint20in, accelConstraint25in)
                .build();

        RedBackstage_RightSpikeToBackdropWaypoint = drive.trajectorySequenceBuilder(RedBackstage_StartToRightSpike.end())
                .lineToLinearHeading(Red_BackdropRightCenterRelocPos, velConstraint40in, accelConstraint70in)
                .build();
        RedBackstage_BackdropRelocWaypointToBackdropRight = drive.trajectorySequenceBuilder(RedBackstage_RightSpikeToBackdropWaypoint.end())
                .lineToLinearHeading(Red_BackdropRightOffsetPos, velConstraint40in, accelConstraint70in)
                .lineToLinearHeading(Red_BackdropRightPos, velConstraint20in, accelConstraint25in)
                .build();

        // backdrop to backdrop
        Red_BackdropLeftToBackdropCenter = drive.trajectorySequenceBuilder(RedBackstage_BackdropRelocWaypointToBackdropLeft.end())
                .forward(4)
                .lineToLinearHeading(Red_BackdropCenterPos, velConstraint40in, accelConstraint70in)
                .build();
        Red_BackdropLeftToBackdropRight = drive.trajectorySequenceBuilder(RedBackstage_BackdropRelocWaypointToBackdropLeft.end())
                .forward(4)
                .lineToLinearHeading(Red_BackdropRightPos, velConstraint40in, accelConstraint70in)
                .build();
        Red_BackdropCenterToBackdropLeft = drive.trajectorySequenceBuilder(RedBackstage_BackdropRelocWaypointToBackdropCenter.end())
                .forward(4)
                .lineToLinearHeading(Red_BackdropLeftPos, velConstraint40in, accelConstraint70in)
                .build();
        Red_BackdropCenterToBackdropRight = drive.trajectorySequenceBuilder(RedBackstage_BackdropRelocWaypointToBackdropCenter.end())
                .forward(4)
                .lineToLinearHeading(Red_BackdropRightPos, velConstraint40in, accelConstraint70in)
                .build();
        Red_BackdropRightToBackdropCenter = drive.trajectorySequenceBuilder(RedBackstage_BackdropRelocWaypointToBackdropRight.end())
                .forward(4)
                .lineToLinearHeading(Red_BackdropCenterPos, velConstraint40in, accelConstraint70in)
                .build();
        Red_BackdropRightToBackdropLeft = drive.trajectorySequenceBuilder(RedBackstage_BackdropRelocWaypointToBackdropRight.end())
                .forward(4)
                .lineToLinearHeading(Red_BackdropLeftPos, velConstraint40in, accelConstraint70in)
                .build();

        // RED PARK
        Red_BackdropLeftToCenterPark = drive.trajectorySequenceBuilder(RedBackstage_BackdropRelocWaypointToBackdropLeft.end())
                .forward(4)
                .lineTo(new Pose2d(Red_CenterParkPos.getX() + 10, Red_CenterParkPos.getY(), Math.toRadians(0)).vec())
                .lineTo(Red_CenterParkPos.vec())
                .build();
        Red_BackdropLeftToCornerPark = drive.trajectorySequenceBuilder(RedBackstage_BackdropRelocWaypointToBackdropLeft.end())
                .forward(4)
                .lineTo(new Pose2d(Red_CornerParkPos.getX() + 10, Red_CornerParkPos.getY(), Math.toRadians(0)).vec())
                .lineTo(Red_CornerParkPos.vec())
                .build();
        Red_BackdropCenterToCenterPark = drive.trajectorySequenceBuilder(RedBackstage_BackdropRelocWaypointToBackdropCenter.end())
                .forward(4)
                .lineTo(new Pose2d(Red_CenterParkPos.getX() + 10, Red_CenterParkPos.getY(), Math.toRadians(0)).vec())
                .lineToLinearHeading(Red_CenterParkPos)
                .build();
        Red_BackdropCenterToCornerPark = drive.trajectorySequenceBuilder(RedBackstage_BackdropRelocWaypointToBackdropCenter.end())
                .forward(4)
                .lineTo(new Pose2d(Red_CornerParkPos.getX() + 10, Red_CornerParkPos.getY(), Math.toRadians(0)).vec())
                .lineTo(Red_CornerParkPos.vec())
                .build();
        Red_BackdropRightToCenterPark = drive.trajectorySequenceBuilder(RedBackstage_BackdropRelocWaypointToBackdropRight.end())
                .forward(4)
                .lineTo(new Pose2d(Red_CenterParkPos.getX() + 10, Red_CenterParkPos.getY(), Math.toRadians(0)).vec())
                .lineTo(Red_CenterParkPos.vec())
                .build();
        Red_BackdropRightToCornerPark = drive.trajectorySequenceBuilder(RedBackstage_BackdropRelocWaypointToBackdropRight.end())
                .forward(4)
                .lineTo(new Pose2d(Red_CornerParkPos.getX() + 10, Red_CornerParkPos.getY(), Math.toRadians(0)).vec())
                .lineTo(Red_CornerParkPos.vec())
                .build();

        Red_BackdropLeftToBackdropWaypointDoor = drive.trajectorySequenceBuilder(RedBackstage_BackdropRelocWaypointToBackdropLeft.end())
                .lineToLinearHeading(Red_DoorBackdropTransitWaypoint)
                .build();
        Red_BackdropRightToBackdropWaypointDoor = drive.trajectorySequenceBuilder(RedBackstage_BackdropRelocWaypointToBackdropRight.end())
                .lineToLinearHeading(Red_DoorBackdropTransitWaypoint)
                .build();
        Red_BackdropCenterToBackdropWaypointDoor = drive.trajectorySequenceBuilder(RedBackstage_BackdropRelocWaypointToBackdropCenter.end())
                .lineToLinearHeading(Red_DoorBackdropTransitWaypoint)
                .build();

        Red_BackdropLeftToBackdropWaypointTruss = drive.trajectorySequenceBuilder(RedBackstage_BackdropRelocWaypointToBackdropLeft.end())
                .lineToLinearHeading(Red_TrussBackdropTransitWaypoint)
                .build();
        Red_BackdropRightToBackdropWaypointTruss = drive.trajectorySequenceBuilder(RedBackstage_BackdropRelocWaypointToBackdropRight.end())
                .lineToLinearHeading(Red_TrussBackdropTransitWaypoint)
                .build();
        Red_BackdropCenterToBackdropWaypointTruss = drive.trajectorySequenceBuilder(RedBackstage_BackdropRelocWaypointToBackdropCenter.end())
                .lineToLinearHeading(Red_TrussBackdropTransitWaypoint)
                .build();

        DoorBackdropWaypointToStackWaypoint = drive.trajectorySequenceBuilder(Red_BackdropCenterToBackdropWaypointDoor.end())
                .splineToSplineHeading(new Pose2d(-18, 11, Math.toRadians(0)), Math.toRadians(0), velConstraint70in, accelConstraint70in)
                .lineToLinearHeading(Red_DoorStackTransitWaypoint, velConstraint70in, accelConstraint40in)
                .build();
        TrussBackdropWaypointToStackWaypoint = drive.trajectorySequenceBuilder(Red_BackdropCenterToBackdropWaypointTruss.end())
                .splineToSplineHeading(new Pose2d(-18, 59, Math.toRadians(0)), Math.toRadians(0), velConstraint70in, accelConstraint70in)
                .lineToLinearHeading(Red_TrussStackTransitWaypoint, velConstraint70in, accelConstraint40in)
                .build();

        // RED CYCLE 🟥🔁

        /** TRANSIT TO BACKSTAGE */ //RedWings_CenterStackToTrussWaypoint.end()
        RedWings_TransitToBackstageViaDoor = drive.trajectorySequenceBuilder(RedWings_CenterStackToDoorWaypoint.end())
                .lineToLinearHeading(Red_CenterParkOffsetPos)
                .build();
        RedWings_TransitToBackstageViaDoorWait = drive.trajectorySequenceBuilder(RedWings_CenterStackToDoorWaypoint.end())
                .waitSeconds(10)
                .lineToLinearHeading(Red_CenterParkOffsetPos)
                .build();

        RedWings_TransitToBackstageViaTruss = drive.trajectorySequenceBuilder(RedWings_CenterStackToTrussWaypoint.end())
                .lineToLinearHeading(Red_CornerParkOffsetPos)
                .build();
        RedWings_TransitToBackstageViaTrussWait = drive.trajectorySequenceBuilder(RedWings_CenterStackToTrussWaypoint.end())
                .waitSeconds(10)
                .lineToLinearHeading(Red_CornerParkOffsetPos)
                .build();

//        Red_BackdropToStackViaTruss = drive.trajectorySequenceBuilder(Red_DoorBackdropTransitWaypointToBackdropCenter.end())
//                .lineToLinearHeading(new Pose2d(56, 59, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(-18, 59, Math.toRadians(0)))
//                .splineToSplineHeading(new Pose2d(-40, 49, Math.toRadians(40)), Math.toRadians(220))
//                .build();
//        Red_BackdropToStackViaDoor = drive.trajectorySequenceBuilder(Red_DoorBackdropTransitWaypointToBackdropCenter.end())
//                .lineToLinearHeading(new Pose2d(56, 12, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(-18, 12, Math.toRadians(0)))
//                .splineToSplineHeading(new Pose2d(-40, 22, Math.toRadians(-40)), Math.toRadians(140))
//                .build();

        DoorStackWaypointToInnerStack = drive.trajectorySequenceBuilder(DoorBackdropWaypointToStackWaypoint.end())
                .back(0.01)
                .lineToLinearHeading(Red_InnerStackPos)
                .back(0.01)
                .build();
        DoorStackWaypointToCenterStack = drive.trajectorySequenceBuilder(DoorBackdropWaypointToStackWaypoint.end())
                .lineToLinearHeading(Red_CenterStackPos)
                .build();
        DoorStackWaypointToOuterStack = drive.trajectorySequenceBuilder(DoorBackdropWaypointToStackWaypoint.end())
                .lineToLinearHeading(Red_OuterStackPos)
                .build();


        Red_BackstageToStackViaTruss = drive.trajectorySequenceBuilder(RedWings_CenterStackToDoorWaypoint.end())
                .lineToLinearHeading(new Pose2d(56, 59, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-48, 59, Math.toRadians(0)))
                .build();
        Red_BackstageToStackViaDoor = drive.trajectorySequenceBuilder(RedWings_CenterStackToDoorWaypoint.end())
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
