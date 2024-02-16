package org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ARTEMIS.trajectorysequence.TrajectorySequence;

public class EXCCMP_AutoTrajectories {

    // blue cycle emojis \uD83D\uDFE6
    // red cycle emojis \uD83D\uDFE5


    //BLUE BACKSTAGE
    public static final Pose2d blueBackstage_StartPos = new Pose2d(-8.5, -64.5, Math.toRadians(90));

    //BLUE WINGS
    public static final Pose2d blueWings_StartPos = new Pose2d(36.5, -64.5, Math.toRadians(90));

    //RED BACKSTAGE
    public static final Pose2d redBackstage_StartPos = new Pose2d(-8.5, 64.5, Math.toRadians(270));

    //RED WINGS
    public static final Pose2d redWings_StartPos = new Pose2d(36.5, 64.5, Math.toRadians(270));

    public static TrajectorySequence RedBackstage_StartToLeftSpike, RedBackstage_StartToCenterSpike, RedBackstage_StartToRightSpike;
    public static TrajectorySequence RedWings_LeftSpikeToStack, RedWings_CenterSpikeToStack, RedWings_RightSpikeToStack;

    public static void generateTrajectories(SampleMecanumDrive drive){

        RedBackstage_StartToLeftSpike =
            drive.trajectorySequenceBuilder(redWings_StartPos)
                    .splineToLinearHeading(new Pose2d(35, 32, Math.toRadians(10)), 180) // left spike
                    .build();

        RedWings_LeftSpikeToStack =
                drive.trajectorySequenceBuilder(RedBackstage_StartToLeftSpike.end())
                        .splineToSplineHeading(new Pose2d(60, 30, 0), 0)
                        .build();

        RedBackstage_StartToCenterSpike =
                drive.trajectorySequenceBuilder(redWings_StartPos)
                        .splineToLinearHeading(new Pose2d(43, 27, Math.toRadians(10)), 180) // center spike
                        .build();

        RedWings_CenterSpikeToStack =
                drive.trajectorySequenceBuilder(RedBackstage_StartToCenterSpike.end())
                        .splineToSplineHeading(new Pose2d(60, 30, 0), 0)
                        .build();

        RedBackstage_StartToRightSpike =
                drive.trajectorySequenceBuilder(redWings_StartPos)
                        .splineToSplineHeading(new Pose2d(60, 30, 0), 0)
                        .build();

        RedWings_RightSpikeToStack = drive.trajectorySequenceBuilder(RedBackstage_StartToRightSpike.end())
                .back(0.01)
                .build();

    }
}
