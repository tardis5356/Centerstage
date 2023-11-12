package org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.CSTB_DriveConstants;
import org.firstinspires.ftc.teamcode.drive.CSTB_SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class CSTB_AutoTrajectories {
    //    public static final Pose2d blue_StartPos = new Pose2d(-30.5, 64, Math.toRadians(90)); 2023-03-04
    public static final Pose2d blue_StartPos = new Pose2d(-40.5, 64, Math.toRadians(90));
    public static final Pose2d blue_MedPreloadPolePos = new Pose2d(-27, 31, Math.toRadians(140));//-35//130
    public static final Pose2d blue_MedPolePos = new Pose2d(-29.5, 21.5, Math.toRadians(220)); //-30.5, 21
    public static final Pose2d blue_StackPos = new Pose2d(-61.5, 13.5, Math.toRadians(180));//-63 //(-59.5, 12)

    public static final Pose2d red_StartPos = new Pose2d(40.5, 64, Math.toRadians(90));
    public static final Pose2d red_MedPreloadPolePos = new Pose2d(30, 29, Math.toRadians(40)); // (30, 16.5) //332 //90
    public static final Pose2d red_MedPolePos = new Pose2d(30.5, 18, Math.toRadians(328)); // (30, 16.5) //332 // (32, 20)  //TODO: -32?
    public static final Pose2d red_StackPos = new Pose2d(62, 12, Math.toRadians(0)); // (62.5, 9)


    //1+4 trajectories
    public static TrajectorySequence blueBackstage_StartToBackstage, blue_PreloadPoleToStackWaypoint, blue_StackWaypointToStack, blue_StackToStackWaypoint, blue_StackWaypointToMedPole, blue_MedPoleToStackWaypoint, blue_MedPreloadPoleToStack, blue_MedPoleToStack, blue_MedPoleToStackSlow;
    public static TrajectorySequence red_StartToPreloadPole, red_PreloadPoleToStackWaypoint, red_StackWaypointToStack, red_StackToStackWaypoint, red_StackWaypointToMedPole, red_MedPoleToStack, red_MedPreloadPoleToStack, red_MedPoleToStackSlow;

    public static void generateTrajectories(CSTB_SampleMecanumDrive drive) {
        // 🟦🟦🟦 BLUE AUTO COMMANDS 🟦🟦🟦
        blueBackstage_StartToBackstage =
                drive.trajectorySequenceBuilder(blue_StartPos)
                        .setReversed(true)
                        .lineTo(new Vector2d(-38, 62), SampleMecanumDrive_R2V2.getVelocityConstraint(86, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .setReversed(false)
                        .build();


        // 🟥🟥🟥 red auto commands 🟥🟥🟥

    }
}