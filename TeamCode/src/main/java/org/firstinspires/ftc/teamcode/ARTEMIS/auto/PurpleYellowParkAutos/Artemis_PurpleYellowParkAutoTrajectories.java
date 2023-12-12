package org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.ARTEMIS.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ARTEMIS.trajectorysequence.TrajectorySequence;

public class Artemis_PurpleYellowParkAutoTrajectories {


    //BLUE BACKSTAGE
    public static final Pose2d blueBackstage_StartPos = new Pose2d(-8.5, -64.5, Math.toRadians(90));
    public static final Pose2d blueBackstage_DecisionPointPos = new Pose2d(-12.5, -48, Math.toRadians(90));
    public static final Pose2d blueBackstage_XXXXSpikeMarkPos = new Pose2d(-14.5, -31, Math.toRadians(90));

    //BLUE WINGS
    public static final Pose2d blueWings_StartPos = new Pose2d(36.5, -64.5, Math.toRadians(90));
    public static final Pose2d blueWings_DecisionPointPos = new Pose2d(40.5, -14, Math.toRadians(270));
    public static final Pose2d blueWings_XXXXSpikeMarkPos = new Pose2d(-14.5, -31, Math.toRadians(90));

    //RED BACKSTAGE
    public static final Pose2d redBackstage_StartPos = new Pose2d(-8.5, 64.5, Math.toRadians(270));
    public static final Pose2d redBackstage_DecisionPointPos = new Pose2d(-12.5, 48, Math.toRadians(270));
    public static final Pose2d redBackstage_BackdropWaypointPos = new Pose2d(-38, 38, Math.toRadians(0));

    //RED WINGS
    public static final Pose2d redWings_StartPos = new Pose2d(36.5, 64.5, Math.toRadians(270));
    public static final Pose2d redWings_DecisionPointPos = new Pose2d(40.5, 14, Math.toRadians(90));
    public static final Pose2d redWings_BackdropWaypointPos = new Pose2d(-38, 38, Math.toRadians(0));

    public static TrajectorySequence blueBackstage_StartPositionToDecisionPoint, blueBackstage_DecisionPointToCenterSpike, blueBackstage_DecisionPointToLeftSpike, blueBackstage_DecisionPointToRightSpike, blueBackstage_LeftSpikeToDecisionPoint, blueBackstage_RightSpikeToDecisionPoint, blueBackstage_CenterSpikeToDecisionPoint, blueBackstage_DecisionPointToBackdropWaypoint, blueBackstage_WaypointToLeftSlots, blueBackstage_WaypointToCenterSlots, blueBackstage_WaypointToRightSlots, blueBackstage_BackdropToCornerPark;
    public static TrajectorySequence blueWings_StartPositionToDecisionPoint, blueWings_DecisionPointToCenterSpike, blueWings_DecisionPointToLeftSpike, blueWings_DecisionPointToRightSpike, blueWings_LeftSpikeToDecisionPoint, blueWings_RightSpikeToDecisionPoint, blueWings_CenterSpikeToDecisionPoint, blueWings_DecisionPointToCornerPark;
    public static TrajectorySequence redBackstage_StartPositionToDecisionPoint, redBackstage_DecisionPointToCenterSpike, redBackstage_DecisionPointToLeftSpike, redBackstage_DecisionPointToRightSpike, redBackstage_LeftSpikeToDecisionPoint, redBackstage_RightSpikeToDecisionPoint, redBackstage_CenterSpikeToDecisionPoint, redBackstage_DecisionPointToBackdropWaypoint, redBackstage_WaypointToLeftSlots, redBackstage_WaypointToCenterSlots, redBackstage_WaypointToRightSlots, redBackstage_LeftSlotsToWaypoint, redBackstage_CenterSlotsToWaypoint, redBackstage_RightSlotsToWaypoint, redBackstage_WaypointToCornerPark;
    public static TrajectorySequence redWings_StartPositionToDecisionPoint, redWings_DecisionPointToCenterSpike, redWings_DecisionPointToLeftSpike, redWings_DecisionPointToRightSpike, redWings_LeftSpikeToDecisionPoint, redWings_RightSpikeToDecisionPoint, redWings_CenterSpikeToDecisionPoint, redWings_DecisionPointToSafetyWaypoint, redWings_SafetyWaypointToBackdropWaypoint, redWings_DecisionPointToBackdropWaypoint, redWings_WaypointToLeftSlots, redWings_WaypointToCenterSlots, redWings_WaypointToRightSlots, redWings_LeftSlotsToWaypoint, redWings_CenterSlotsToWaypoint, redWings_RightSlotsToWaypoint, redWings_BackdropToCenterPark;
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

            blueBackstage_DecisionPointToLeftSpike = //shift and go forward to score left
                    drive.trajectorySequenceBuilder(blueBackstage_DecisionPointPos)
                            .forward(4)
                            .turn(Math.toRadians(45))
                            .build();

            blueBackstage_LeftSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(blueBackstage_DecisionPointToLeftSpike.end())
                            .lineToLinearHeading(blueBackstage_DecisionPointPos)
                            .build();

            blueBackstage_DecisionPointToCenterSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(blueBackstage_DecisionPointPos)
                            .forward(18)
                            .back(7)
                            .build();

            blueBackstage_CenterSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(blueBackstage_DecisionPointToCenterSpike.end())
                            .lineToLinearHeading(blueBackstage_DecisionPointPos)
                            .build();

            blueBackstage_DecisionPointToRightSpike = //shift and go forward to score right
                    drive.trajectorySequenceBuilder(blueBackstage_DecisionPointPos)
                            .forward(2)
                            .turn(Math.toRadians(-45))
                            .forward(14)
                            .back(4)
                            .build();

            blueBackstage_RightSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(blueBackstage_DecisionPointToRightSpike.end())
                            .lineToLinearHeading(blueBackstage_DecisionPointPos)
                            .build();

            blueBackstage_DecisionPointToBackdropWaypoint = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(blueBackstage_DecisionPointPos)
                            .back(4)
                            .lineToLinearHeading(new Pose2d(-38, -38, Math.toRadians(0)))
                            .build();

            blueBackstage_WaypointToLeftSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(blueBackstage_DecisionPointToBackdropWaypoint.end())
                            .lineToLinearHeading(new Pose2d(-50, -44, Math.toRadians(0)))
                            .build();

            blueBackstage_WaypointToCenterSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(blueBackstage_DecisionPointToBackdropWaypoint.end())
                            .lineToLinearHeading(new Pose2d(-50.25, -37, Math.toRadians(0)))
                            .build();

            blueBackstage_WaypointToRightSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(blueBackstage_DecisionPointToBackdropWaypoint.end())
                            .lineToLinearHeading(new Pose2d(-49.5, -32, Math.toRadians(0)))
                            .build();

            blueBackstage_BackdropToCornerPark = //go to and spin to parking between backdrops
                    drive.trajectorySequenceBuilder(blueBackstage_WaypointToCenterSlots.end())
                            .forward(4)
                            .strafeRight(24)
                            .lineToLinearHeading(new Pose2d(-56, -62, Math.toRadians(0)))
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
                            .forward(2)
                            .turn(Math.toRadians(45))
                            .forward(14)
                            .back(5)
                            .build();

            redBackstage_LeftSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(redBackstage_DecisionPointToLeftSpike.end())
                            .lineToLinearHeading(redBackstage_DecisionPointPos)
                            .build();

            redBackstage_DecisionPointToCenterSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(redBackstage_DecisionPointPos)
                            .forward(18)
                            .back(6)
                            .build();

            redBackstage_CenterSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(redBackstage_DecisionPointToCenterSpike.end())
                            .lineToLinearHeading(redBackstage_DecisionPointPos)
                            .build();

            redBackstage_DecisionPointToRightSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(redBackstage_DecisionPointPos)
                            .forward(6)
                            .turn(Math.toRadians(-50))
                            .build();

            redBackstage_RightSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(redBackstage_DecisionPointToRightSpike.end())
                            .lineToLinearHeading(redBackstage_DecisionPointPos)
                            .build();

            redBackstage_DecisionPointToBackdropWaypoint = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(redBackstage_DecisionPointPos)
                            .lineToLinearHeading(redBackstage_BackdropWaypointPos)
                            .build();

            redBackstage_WaypointToLeftSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(redBackstage_DecisionPointToBackdropWaypoint.end())
                            .lineToLinearHeading(new Pose2d(-50.2, 32, Math.toRadians(0)))
                            .build();

            redBackstage_WaypointToCenterSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(redBackstage_DecisionPointToBackdropWaypoint.end())
                            .lineToLinearHeading(new Pose2d(-49.5, 39, Math.toRadians(0)))
                            .build();

            redBackstage_WaypointToRightSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(redBackstage_DecisionPointToBackdropWaypoint.end())
                            .lineToLinearHeading(new Pose2d(-49.5, 46, Math.toRadians(0)))
                            .build();

            redBackstage_LeftSlotsToWaypoint =
                    drive.trajectorySequenceBuilder(redBackstage_DecisionPointToBackdropWaypoint.end())
                            .forward(4)
                            .lineTo(redBackstage_BackdropWaypointPos.vec())
                            .build();

            redBackstage_CenterSlotsToWaypoint =
                    drive.trajectorySequenceBuilder(redBackstage_DecisionPointToBackdropWaypoint.end())
                            .forward(4)
                            .lineTo(redBackstage_BackdropWaypointPos.vec())
                            .build();

            redBackstage_RightSlotsToWaypoint =
                    drive.trajectorySequenceBuilder(redBackstage_DecisionPointToBackdropWaypoint.end())
                            .forward(4)
                            .lineTo(redBackstage_BackdropWaypointPos.vec())
                            .build();

            redBackstage_WaypointToCornerPark = //go to and spin to parking between backdrops
                    drive.trajectorySequenceBuilder(redBackstage_BackdropWaypointPos)
                            .strafeLeft(24)
                            .lineToLinearHeading(new Pose2d(-56, 64, Math.toRadians(0)))
                            .build();
        }

        //red wings
        {
            redWings_StartPositionToDecisionPoint = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(redWings_StartPos)
                            .lineTo(new Vector2d(40.5, 64.5), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .forward(6)
                            .turn(Math.toRadians(180))
                            .lineTo(redWings_DecisionPointPos.vec(), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();

            redWings_DecisionPointToLeftSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(redWings_DecisionPointPos)
                            .forward(2)
                            .turn(Math.toRadians(45))
                            .forward(14)
                            .back(5)
                            .build();

            redWings_LeftSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(redWings_DecisionPointToLeftSpike.end())
                            .lineToLinearHeading(redWings_DecisionPointPos)
                            .build();

            redWings_DecisionPointToCenterSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(redWings_DecisionPointPos)
                            .forward(2)
                            .build();

            redWings_CenterSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(redWings_DecisionPointToCenterSpike.end())
                            .lineToLinearHeading(redWings_DecisionPointPos)
                            .build();

            redWings_DecisionPointToRightSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(redWings_DecisionPointPos)
                            .forward(6)
                            .turn(Math.toRadians(-50))
                            .build();

            redWings_RightSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(redWings_DecisionPointToRightSpike.end())
                            .lineToLinearHeading(redWings_DecisionPointPos)
                            .build();

            redWings_DecisionPointToSafetyWaypoint = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(redWings_DecisionPointPos)
                            .turn(Math.toRadians(-90))
                            .lineToLinearHeading(new Pose2d(-38, 12, Math.toRadians(0)))
                            .build();

            redWings_SafetyWaypointToBackdropWaypoint = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(redWings_DecisionPointToSafetyWaypoint.end())
                            .lineToLinearHeading(redWings_BackdropWaypointPos)
                            .build();

            redWings_DecisionPointToBackdropWaypoint = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(redWings_DecisionPointPos)
                            .lineToLinearHeading(redWings_BackdropWaypointPos)
                            .build();

            redWings_WaypointToLeftSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(redWings_SafetyWaypointToBackdropWaypoint.end())
                            .lineToLinearHeading(new Pose2d(-50, 32, Math.toRadians(0)))
                            .build();

            redWings_WaypointToCenterSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(redWings_SafetyWaypointToBackdropWaypoint.end())
                            .lineToLinearHeading(new Pose2d(-48, 41, Math.toRadians(0)))
                            .build();

            redWings_WaypointToRightSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(redWings_SafetyWaypointToBackdropWaypoint.end())
                            .lineToLinearHeading(new Pose2d(-49.5, 46, Math.toRadians(0)))
                            .build();

            redWings_LeftSlotsToWaypoint =
                    drive.trajectorySequenceBuilder(redWings_DecisionPointToBackdropWaypoint.end())
                            .forward(4)
                            .lineTo(redWings_BackdropWaypointPos.vec())
                            .build();

            redWings_CenterSlotsToWaypoint =
                    drive.trajectorySequenceBuilder(redWings_DecisionPointToBackdropWaypoint.end())
                            .forward(4)
                            .lineTo(redWings_BackdropWaypointPos.vec())
                            .build();

            redWings_RightSlotsToWaypoint =
                    drive.trajectorySequenceBuilder(redWings_DecisionPointToBackdropWaypoint.end())
                            .forward(4)
                            .lineTo(redWings_BackdropWaypointPos.vec())
                            .build();

            redWings_BackdropToCenterPark = //go to and spin to parking between backdrops
                    drive.trajectorySequenceBuilder(redWings_WaypointToCenterSlots.end())
                            .forward(4)
                            .strafeLeft(24)
                            .lineToLinearHeading(new Pose2d(-56, 62, Math.toRadians(0)))
                            .build();
        }

    }
    }
