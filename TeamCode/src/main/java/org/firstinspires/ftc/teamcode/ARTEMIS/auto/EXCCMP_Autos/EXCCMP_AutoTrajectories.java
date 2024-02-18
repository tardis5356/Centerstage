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
            velConstraint40in = SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
            velConstraint20in = SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
            velConstraint10in = SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    public static final TrajectoryAccelerationConstraint
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

    public static TrajectorySequence RedWings_StartToOrigin, RedBackstage_StartToOrigin, BlueWings_StartToOrigin, BlueBackstage_StartToOrigin;

    public static TrajectorySequence RedWings_StartToLeftSpike, RedWings_StartToCenterSpike, RedWings_StartToRightSpike;
    public static TrajectorySequence RedWings_LeftSpikeToStack, RedWings_CenterSpikeToStack, RedWings_RightSpikeToStack;
    public static TrajectorySequence RedWings_StackPickupSequence;
    public static TrajectorySequence RedWings_TransitToBackstageViaDoor, RedWings_TransitToBackstageViaTruss, RedWings_TransitToBackdropViaDoor, RedWings_TransitToBackdropViaTruss;
    public static TrajectorySequence RedWings_TransitToBackstageViaDoorWait, RedWings_TransitToBackstageViaTrussWait, RedWings_TransitToBackdropViaDoorWait, RedWings_TransitToBackdropViaTrussWait;

    public static TrajectorySequence RedBackstage_StartToLeftSpike, RedBackstage_StartToCenterSpike, RedBackstage_StartToRightSpike;
    public static TrajectorySequence RedBackstage_SpikeToBackdrop;

    public static TrajectorySequence Red_BackdropToStackViaTruss, Red_BackdropToStackViaDoor, Red_BackstageToStackViaTruss, Red_BackstageToStackViaDoor;


    public static void generateTrajectories(SampleMecanumDrive drive) {

        // redwings 🟥🟥🟥🟥
        // red wings 🟥🪽 start 🏁 to spikes 🌵
        RedWings_StartToLeftSpike =
                drive.trajectorySequenceBuilder(redWings_StartPos)
                        .splineToLinearHeading(new Pose2d(35, 32, Math.toRadians(10)), 180) // left spike
                        .build();

        RedWings_LeftSpikeToStack =
                drive.trajectorySequenceBuilder(RedWings_StartToLeftSpike.end())
                        .splineToSplineHeading(new Pose2d(60, 30, 0), 0)
                        .build();

        RedWings_StartToCenterSpike =
                drive.trajectorySequenceBuilder(redWings_StartPos)
                        .splineToLinearHeading(new Pose2d(43, 27, Math.toRadians(10)), 180) // center spike
                        .build();

        // redwings 🟥🪽 spike 🌵 to stack 🥞
        RedWings_CenterSpikeToStack =
                drive.trajectorySequenceBuilder(RedWings_StartToCenterSpike.end())
                        .splineToSplineHeading(new Pose2d(60, 30, 0), 0)
                        .build();

        RedWings_StartToRightSpike =
                drive.trajectorySequenceBuilder(redWings_StartPos)
                        .splineToSplineHeading(new Pose2d(60, 30, 0), 0)
                        .build();

        RedWings_RightSpikeToStack = drive.trajectorySequenceBuilder(RedWings_StartToRightSpike.end())
                .back(0.01)
                .build();

        // redWings 🟥🪽 stack pickup 🥞🥞
        RedWings_StackPickupSequence = drive.trajectorySequenceBuilder(RedWings_StartToRightSpike.end())
                .setConstraints(velConstraint10in, accelConstraint40in)
                .strafeLeft(4)
                .waitSeconds(0.5)
                .back(4)
                .setConstraints(velConstraint70in, accelConstraint70in)
                .build();

        // redWings 🟥🪽 transit to back 🎭
        RedWings_TransitToBackdropViaDoor = drive.trajectorySequenceBuilder(RedWings_StackPickupSequence.end())
                .lineToLinearHeading(new Pose2d(56, 12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-18, 12, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-40, 22, Math.toRadians(-40)), Math.toRadians(140))
                .build();
        RedWings_TransitToBackdropViaDoorWait = drive.trajectorySequenceBuilder(RedWings_StackPickupSequence.end())
                .lineToLinearHeading(new Pose2d(56, 12, Math.toRadians(0)))
                .waitSeconds(10)
                .lineToLinearHeading(new Pose2d(-18, 12, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-40, 22, Math.toRadians(-40)), Math.toRadians(140))
                .build();

        RedWings_TransitToBackdropViaTruss = drive.trajectorySequenceBuilder(RedWings_StackPickupSequence.end())
                .lineToLinearHeading(new Pose2d(56, 59, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-18, 59, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-40, 49, Math.toRadians(40)), Math.toRadians(220))
                .build();
        RedWings_TransitToBackdropViaTrussWait = drive.trajectorySequenceBuilder(RedWings_StackPickupSequence.end())
                .lineToLinearHeading(new Pose2d(56, 59, Math.toRadians(0)))
                .waitSeconds(10)
                .lineToLinearHeading(new Pose2d(-18, 59, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-40, 49, Math.toRadians(40)), Math.toRadians(220))
                .build();

        RedWings_TransitToBackstageViaDoor = drive.trajectorySequenceBuilder(RedWings_StackPickupSequence.end())
                .lineToLinearHeading(new Pose2d(56, 12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-48, 12, Math.toRadians(0)))
                .build();
        RedWings_TransitToBackstageViaDoor = drive.trajectorySequenceBuilder(RedWings_StackPickupSequence.end())
                .lineToLinearHeading(new Pose2d(56, 12, Math.toRadians(0)))
                .waitSeconds(10)
                .lineToLinearHeading(new Pose2d(-48, 12, Math.toRadians(0)))
                .build();

        RedWings_TransitToBackstageViaTruss = drive.trajectorySequenceBuilder(RedWings_StackPickupSequence.end())
                .lineToLinearHeading(new Pose2d(56, 59, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-48, 59, Math.toRadians(0)))
                .build();

        RedWings_TransitToBackstageViaTrussWait = drive.trajectorySequenceBuilder(RedWings_StackPickupSequence.end())
                .lineToLinearHeading(new Pose2d(56, 59, Math.toRadians(0)))
                .waitSeconds(10)
                .lineToLinearHeading(new Pose2d(-48, 59, Math.toRadians(0)))
                .build();

        // RED BACKSTAGE 🟥🟥🟥🟥
        // redBackstage 🟥🎭 start 🏁 to spike 🌵
        RedBackstage_StartToLeftSpike = drive.trajectorySequenceBuilder(redBackstage_StartPos)
                .splineToLinearHeading(new Pose2d(-12, 30, Math.toRadians(180)), Math.toRadians(290))
                .build();
        RedBackstage_StartToLeftSpike = drive.trajectorySequenceBuilder(redBackstage_StartPos)
                .splineToLinearHeading(new Pose2d(-25, 27, Math.toRadians(160)), 180) // center spike
                .build();
        RedBackstage_StartToLeftSpike = drive.trajectorySequenceBuilder(redBackstage_StartPos)
                .splineToSplineHeading(new Pose2d(-35, 34, Math.toRadians(180)), Math.toRadians(180)) // left spike
                .build();

        // redBackstage 🟥🎭 spike 🌵 to backdrop 🎭
        RedBackstage_SpikeToBackdrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-40, 34, Math.toRadians(0)))
                .build();


        // RED CYCLE 🟥🔁
        Red_BackdropToStackViaTruss = drive.trajectorySequenceBuilder(RedWings_StackPickupSequence.end())
                .lineToLinearHeading(new Pose2d(56, 59, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-18, 59, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-40, 49, Math.toRadians(40)), Math.toRadians(220))
                .build();
        Red_BackdropToStackViaDoor = drive.trajectorySequenceBuilder(RedWings_StackPickupSequence.end())
                .lineToLinearHeading(new Pose2d(56, 12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-18, 12, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-40, 22, Math.toRadians(-40)), Math.toRadians(140))
                .build();

        Red_BackstageToStackViaTruss = drive.trajectorySequenceBuilder(RedWings_StackPickupSequence.end())
                .lineToLinearHeading(new Pose2d(56, 59, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-48, 59, Math.toRadians(0)))
                .build();
        Red_BackstageToStackViaDoor = drive.trajectorySequenceBuilder(RedWings_StackPickupSequence.end())
                .lineToLinearHeading(new Pose2d(56, 12, Math.toRadians(0)))
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
