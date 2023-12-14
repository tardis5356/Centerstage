package org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.ARTEMIS.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ARTEMIS.trajectorysequence.TrajectorySequence;

public class Artemis_PurpleYellowParkAutoTrajectories {


    //BLUE BACKSTAGE
    public static final Pose2d blueBackstage_StartPos = new Pose2d(-12.5, -64.5, Math.toRadians(90));
    public static final Pose2d blueBackstage_DecisionPointPos = new Pose2d(-12.5, -48, Math.toRadians(90));

    //BLUE WINGS
    public static final Pose2d blueWings_StartPos = new Pose2d(36.5, -64.5, Math.toRadians(90));
    public static final Pose2d blueWings_DecisionPointPos = new Pose2d(40.5, -13, Math.toRadians(270));


    //GENERAL BLUE
    public static final Pose2d blue_BackdropWaypointPos = new Pose2d(-38, -34, Math.toRadians(0));

    //RED BACKSTAGE
    public static final Pose2d redBackstage_StartPos = new Pose2d(-8.5, 64.5, Math.toRadians(270));
    public static final Pose2d redBackstage_DecisionPointPos = new Pose2d(-12.5, 48, Math.toRadians(270));
    public static final Pose2d redBackstage_BackdropWaypointPos = new Pose2d(-38, 38, Math.toRadians(0));

    //RED WINGS
    public static final Pose2d redWings_StartPos = new Pose2d(36.5, 64.5, Math.toRadians(270));
    public static final Pose2d redWings_DecisionPointPos = new Pose2d(40.5, 14, Math.toRadians(90));
    public static final Pose2d redWings_BackdropWaypointPos = new Pose2d(-38, 38, Math.toRadians(0));

    public static TrajectorySequence blueBackstage_StartPositionToDecisionPoint, blueBackstage_DecisionPointToCenterSpike, blueBackstage_DecisionPointToLeftSpike, blueBackstage_DecisionPointToRightSpike, blueBackstage_LeftSpikeToDecisionPoint, blueBackstage_RightSpikeToDecisionPoint, blueBackstage_CenterSpikeToDecisionPoint, blueBackstage_DecisionPointToBackdropWaypoint;
    public static TrajectorySequence blueWings_StartPositionToDecisionPoint, blueWings_DecisionPointToCenterSpike, blueWings_DecisionPointToLeftSpike, blueWings_DecisionPointToRightSpike, blueWings_LeftSpikeToDecisionPoint, blueWings_RightSpikeToDecisionPoint, blueWings_CenterSpikeToDecisionPoint, blueWings_DecisionPointToSafetyWaypoint, blueWings_SafetyWaypointToBackdropWaypoint, blueWings_DecisionPointToBackdropWaypoint, blueWings_WaypointToLeftSlots, blueWings_WaypointToCenterSlots, blueWings_WaypointToRightSlots, blueWings_LeftSlotsToWaypoint, blueWings_CenterSlotsToWaypoint, blueWings_RightSlotsToWaypoint;
    public static TrajectorySequence blue_WaypointToLeftSlots, blue_WaypointToCenterSlots, blue_WaypointToRightSlots, blue_LeftSlotsToBackdropWaypoint, blue_CenterSlotsToBackdropWaypoint, blue_RightSlotsToBackdropWaypoint,  blue_BackdropToCornerPark, blue_BackdropToCenterPark;
    public static TrajectorySequence redBackstage_StartPositionToDecisionPoint, redBackstage_DecisionPointToCenterSpike, redBackstage_DecisionPointToLeftSpike, redBackstage_DecisionPointToRightSpike, redBackstage_LeftSpikeToDecisionPoint, redBackstage_RightSpikeToDecisionPoint, redBackstage_CenterSpikeToDecisionPoint, redBackstage_DecisionPointToBackdropWaypoint, redBackstage_WaypointToLeftSlots, redBackstage_WaypointToCenterSlots, redBackstage_WaypointToRightSlots, redBackstage_LeftSlotsToWaypoint, redBackstage_CenterSlotsToWaypoint, redBackstage_RightSlotsToWaypoint, redBackstage_WaypointToCornerPark;
    public static TrajectorySequence redWings_StartPositionToDecisionPoint, redWings_DecisionPointToCenterSpike, redWings_DecisionPointToLeftSpike, redWings_DecisionPointToRightSpike, redWings_LeftSpikeToDecisionPoint, redWings_RightSpikeToDecisionPoint, redWings_CenterSpikeToDecisionPoint, redWings_DecisionPointToSafetyWaypoint, redWings_SafetyWaypointToBackdropWaypoint, redWings_DecisionPointToBackdropWaypoint, redWings_WaypointToLeftSlots, redWings_WaypointToCenterSlots, redWings_WaypointToRightSlots, redWings_LeftSlotsToWaypoint, redWings_CenterSlotsToWaypoint, redWings_RightSlotsToWaypoint, redWings_BackdropToCenterPark;
    public static void generateTrajectories(SampleMecanumDrive drive) {

        //blue backstage //44.13/16ths
        {
            blueBackstage_StartPositionToDecisionPoint = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(blueBackstage_StartPos)
                            .lineTo(new Vector2d(-12.51, -64.5), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
        }

        //blue wings
        {
            blueWings_StartPositionToDecisionPoint = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(blueWings_StartPos)
                            .lineTo(new Vector2d(40.5, -64.5), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .forward(6)
                            .turn(Math.toRadians(180))
                            .lineTo(blueWings_DecisionPointPos.vec(), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();

            blueWings_DecisionPointToLeftSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(blueWings_DecisionPointPos)
                            .forward(12)
                            .turn(Math.toRadians(-90))
                            .forward(2)
                            .build();

            blueWings_LeftSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(blueWings_DecisionPointToLeftSpike.end())
                            .lineToLinearHeading(blueWings_DecisionPointPos)
                            .build();

            blueWings_DecisionPointToCenterSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(blueWings_DecisionPointPos)
                            .forward(3)
                            .build();

            blueWings_CenterSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(blueWings_DecisionPointToCenterSpike.end())
                            .lineToLinearHeading(blueWings_DecisionPointPos)
                            .build();

            blueWings_DecisionPointToRightSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(blueWings_DecisionPointPos)
                            .forward(2)
                            .turn(Math.toRadians(35))
                            .forward(12)
                            .back(6)
                            .build();

            blueWings_RightSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(blueWings_DecisionPointToRightSpike.end())
                            .lineToLinearHeading(blueWings_DecisionPointPos)
                            .build();

            blueWings_DecisionPointToSafetyWaypoint = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(blueWings_DecisionPointPos)
                            .turn(Math.toRadians(90))
                            .lineToLinearHeading(new Pose2d(-38, -13, Math.toRadians(0)))
                            .build();

            blueWings_SafetyWaypointToBackdropWaypoint = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(blueWings_DecisionPointToSafetyWaypoint.end())
                            .lineToLinearHeading(blue_BackdropWaypointPos)
                            .build();

            blueWings_DecisionPointToBackdropWaypoint = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(blueWings_DecisionPointPos)
                            .lineToLinearHeading(blue_BackdropWaypointPos)
                            .build();

        }

        //general blue
        {
            blue_WaypointToLeftSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(blue_BackdropWaypointPos)
                            .lineToLinearHeading(new Pose2d(-48.625, -37, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))
                            .build();

            blue_WaypointToCenterSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(blue_BackdropWaypointPos)
                            .lineToLinearHeading(new Pose2d(-47.5, -34.5, Math.toRadians(0)))
                            .build();

            blue_WaypointToRightSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(blue_BackdropWaypointPos)
                            .lineToLinearHeading(new Pose2d(-48.5, -26, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))
                            .build();

            blue_LeftSlotsToBackdropWaypoint =
                    drive.trajectorySequenceBuilder(blue_WaypointToLeftSlots.end())
                            .forward(3)
                            .lineTo(blue_BackdropWaypointPos.vec())
                            .build();

            blue_CenterSlotsToBackdropWaypoint =
                    drive.trajectorySequenceBuilder(blue_WaypointToCenterSlots.end())
                            .forward(3)
                            .lineTo(blue_BackdropWaypointPos.vec())
                            .build();

            blue_RightSlotsToBackdropWaypoint =
                    drive.trajectorySequenceBuilder(blue_WaypointToRightSlots.end())
                            .forward(3)
                            .lineTo(blue_BackdropWaypointPos.vec())
                            .build();

            blue_BackdropToCornerPark = //go to and spin to parking between backdrops
                    drive.trajectorySequenceBuilder(blue_BackdropWaypointPos)
                            .strafeRight(24)
                            .lineToLinearHeading(new Pose2d(-56, -62, Math.toRadians(0)))
                            .build();

            blue_BackdropToCenterPark =//go to and spin to parking between backdrops
                    drive.trajectorySequenceBuilder(blue_BackdropWaypointPos)
                            .strafeLeft(25)
                            .lineToLinearHeading(new Pose2d(-52, -8, Math.toRadians(0)))
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
