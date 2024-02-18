package org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleYellowParkAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.ARTEMIS.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ARTEMIS.trajectorysequence.TrajectorySequence;

public class OLD_Artemis_PurpleYellowParkAutoTrajectories {


    //BLUE BACKSTAGE
    public static final Pose2d OLD_blueBackstage_StartPos = new Pose2d(-8.5, -64.5, Math.toRadians(90));
    public static final Pose2d OLD_blueBackstage_DecisionPointPos = new Pose2d(-12.5, -48, Math.toRadians(90));
    public static final Pose2d OLD_blueBackstage_BackdropWaypointPos = new Pose2d(-38, -38, Math.toRadians(0));


    //BLUE WINGS
    public static final Pose2d OLD_blueWings_StartPos = new Pose2d(36.5, -64.5, Math.toRadians(90));
    public static final Pose2d OLD_blueWings_DecisionPointPos = new Pose2d(40.5, -13, Math.toRadians(270));
    public static final Pose2d OLD_blueWings_BackdropWaypointPos = new Pose2d(-32, -34, Math.toRadians(0));

    //RED BACKSTAGE
    public static final Pose2d OLD_redBackstage_StartPos = new Pose2d(-8.5, 64.5, Math.toRadians(270));
    public static final Pose2d OLD_redBackstage_DecisionPointPos = new Pose2d(-12.5, 48, Math.toRadians(270));
    public static final Pose2d OLD_redBackstage_BackdropWaypointPos = new Pose2d(-38, 38, Math.toRadians(0));

    //RED WINGS
    public static final Pose2d OLD_redWings_StartPos = new Pose2d(36.5, 64.5, Math.toRadians(270));
    public static final Pose2d OLD_redWings_DecisionPointPos = new Pose2d(40.5, 11, Math.toRadians(90));
    public static final Pose2d OLD_redWings_BackdropWaypointPos = new Pose2d(-38, 38, Math.toRadians(0));

    public static TrajectorySequence OLD_blueBackstage_StartPositionToDecisionPoint, OLD_blueBackstage_DecisionPointToCenterSpike, OLD_blueBackstage_DecisionPointToLeftSpike, OLD_blueBackstage_DecisionPointToRightSpike, OLD_blueBackstage_LeftSpikeToDecisionPoint, OLD_blueBackstage_RightSpikeToDecisionPoint, OLD_blueBackstage_CenterSpikeToDecisionPoint, OLD_blueBackstage_DecisionPointToBackdropWaypoint, OLD_blueBackstage_WaypointToLeftSlots, OLD_blueBackstage_WaypointToCenterSlots, OLD_blueBackstage_WaypointToRightSlots, OLD_blueBackstage_LeftSlotsToBackdropWaypoint, OLD_blueBackstage_CenterSlotsToBackdropWaypoint, OLD_blueBackstage_RightSlotsToBackdropWaypoint, OLD_blueBackstage_BackdropToCornerPark, OLD_blueBackstage_BackdropToCenterPark;
    public static TrajectorySequence OLD_blueWings_StartPositionToDecisionPoint, OLD_blueWings_DecisionPointToCenterSpike, OLD_blueWings_DecisionPointToLeftSpike, OLD_blueWings_DecisionPointToRightSpike, OLD_blueWings_LeftSpikeToDecisionPoint, OLD_blueWings_RightSpikeToDecisionPoint, OLD_blueWings_CenterSpikeToDecisionPoint, OLD_blueWings_DecisionPointToSafetyWaypoint, OLD_blueWings_SafetyWaypointToBackdropWaypoint, OLD_blueWings_DecisionPointToBackdropWaypoint, OLD_blueWings_WaypointToLeftSlots, OLD_blueWings_WaypointToCenterSlots, OLD_blueWings_WaypointToRightSlots, OLD_blueWings_LeftSlotsToBackdropWaypoint, OLD_blueWings_CenterSlotsToBackdropWaypoint, OLD_blueWings_RightSlotsToBackdropWaypoint, OLD_blueWings_BackdropToCenterPark, OLD_blueWings_BackdropToCornerPark;
    public static TrajectorySequence OLD_redBackstage_StartPositionToDecisionPoint, OLD_redBackstage_DecisionPointToCenterSpike, OLD_redBackstage_DecisionPointToLeftSpike, OLD_redBackstage_DecisionPointToRightSpike, OLD_redBackstage_LeftSpikeToDecisionPoint, OLD_redBackstage_RightSpikeToDecisionPoint, OLD_redBackstage_CenterSpikeToDecisionPoint, OLD_redBackstage_DecisionPointToBackdropWaypoint, OLD_redBackstage_WaypointToLeftSlots, OLD_redBackstage_WaypointToCenterSlots, OLD_redBackstage_WaypointToRightSlots, OLD_redBackstage_LeftSlotsToBackdropWaypoint, OLD_redBackstage_CenterSlotsToBackdropWaypoint, OLD_redBackstage_RightSlotsToBackdropWaypoint, OLD_redBackstage_WaypointToCornerPark, OLD_redBackstage_WaypointToCenterPark;
    public static TrajectorySequence OLD_redWings_StartPositionToDecisionPoint, OLD_redWings_DecisionPointToCenterSpike, OLD_redWings_DecisionPointToLeftSpike, OLD_redWings_DecisionPointToRightSpike, OLD_redWings_LeftSpikeToDecisionPoint, OLD_redWings_RightSpikeToDecisionPoint, OLD_redWings_CenterSpikeToDecisionPoint, OLD_redWings_DecisionPointToSafetyWaypoint, OLD_redWings_SafetyWaypointToBackdropWaypoint, OLD_redWings_DecisionPointToBackdropWaypoint, OLD_redWings_WaypointToLeftSlots, OLD_redWings_WaypointToCenterSlots, OLD_redWings_WaypointToRightSlots, OLD_redWings_LeftSlotsToBackdropWaypoint, OLD_redWings_CenterSlotsToBackdropWaypoint, OLD_redWings_RightSlotsToBackdropWaypoint, OLD_redWings_BackdropToCenterPark, OLD_redWings_BackdropToCornerPark;

    public static void generateTrajectories(SampleMecanumDrive drive) {

        //blue backstage //44.13/16ths
        {
            OLD_blueBackstage_StartPositionToDecisionPoint = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(OLD_blueBackstage_StartPos)
                            .lineTo(new Vector2d(-12.5, -64.5), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .lineTo(OLD_blueBackstage_DecisionPointPos.vec(), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();

            OLD_blueBackstage_DecisionPointToLeftSpike = //shift and go forward to score left
                    drive.trajectorySequenceBuilder(OLD_blueBackstage_DecisionPointPos)
                            .forward(8)
                            .turn(Math.toRadians(45))
                            .forward(4)
                            .back(4)
                            .build();

            OLD_blueBackstage_LeftSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(OLD_blueBackstage_DecisionPointToLeftSpike.end())
                            .lineToLinearHeading(OLD_blueBackstage_DecisionPointPos)
                            .build();

            OLD_blueBackstage_DecisionPointToCenterSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(OLD_blueBackstage_DecisionPointPos)
                            .forward(18)
                            .back(6)
                            .build();

            OLD_blueBackstage_CenterSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(OLD_blueBackstage_DecisionPointToCenterSpike.end())
                            .lineToLinearHeading(OLD_blueBackstage_DecisionPointPos)
                            .build();

            OLD_blueBackstage_DecisionPointToRightSpike = //shift and go forward to score right
                    drive.trajectorySequenceBuilder(OLD_blueBackstage_DecisionPointPos)
                            .forward(2)
                            .turn(Math.toRadians(-45))
                            .forward(14)
                            .back(4)
                            .build();

            OLD_blueBackstage_RightSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(OLD_blueBackstage_DecisionPointToRightSpike.end())
                            .lineToLinearHeading(OLD_blueBackstage_DecisionPointPos)
                            .build();

            OLD_blueBackstage_DecisionPointToBackdropWaypoint = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(OLD_blueBackstage_DecisionPointPos)
                            .back(8)
                            .lineToLinearHeading(new Pose2d(-38, -38, Math.toRadians(0)))
                            .build();

            OLD_blueBackstage_WaypointToLeftSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(OLD_blueBackstage_BackdropWaypointPos)
                            .lineToLinearHeading(new Pose2d(-50.35, -42, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))
                            .build();

            OLD_blueBackstage_WaypointToCenterSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(OLD_blueBackstage_BackdropWaypointPos)
                            .lineToLinearHeading(new Pose2d(-50.65, -35, Math.toRadians(0)))
                            .build();

            OLD_blueBackstage_WaypointToRightSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(OLD_blueBackstage_BackdropWaypointPos)
                            .lineToLinearHeading(new Pose2d(-51.5, -27.5, Math.toRadians(0)))
                            .build();

            OLD_blueBackstage_LeftSlotsToBackdropWaypoint =
                    drive.trajectorySequenceBuilder(OLD_blueBackstage_WaypointToLeftSlots.end())
                            .forward(3)
                            .lineTo(OLD_blueBackstage_BackdropWaypointPos.vec(), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(30))
                            .build();

            OLD_blueBackstage_CenterSlotsToBackdropWaypoint =
                    drive.trajectorySequenceBuilder(OLD_blueBackstage_WaypointToCenterSlots.end())
                            .forward(3)
                            .lineTo(OLD_blueBackstage_BackdropWaypointPos.vec(), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(30))
                            .build();

            OLD_blueBackstage_RightSlotsToBackdropWaypoint =
                    drive.trajectorySequenceBuilder(OLD_blueBackstage_WaypointToRightSlots.end())
                            .forward(3)
                            .lineTo(OLD_blueBackstage_BackdropWaypointPos.vec(), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(30))
                            .build();

            OLD_blueBackstage_BackdropToCornerPark = //go to and spin to parking between backdrops
                    drive.trajectorySequenceBuilder(OLD_blueBackstage_BackdropWaypointPos)
                            .strafeRight(23)
                            .turn(Math.toRadians(-90))
                            .strafeRight(20)
//                            .turn(Math.toRadians(90))
//                            .lineToLinearHeading(new Pose2d(-68, -56, Math.toRadians(0)))
//                            .back(24)
                            .build();

            OLD_blueBackstage_BackdropToCenterPark = //go to and spin to parking between backdrops
                    drive.trajectorySequenceBuilder(OLD_blueBackstage_BackdropWaypointPos)
                            .strafeLeft(28)
                            .turn(Math.toRadians(-90))
                            .strafeRight(20)
//                            .turn(Math.toRadians(90))
//                            .lineToLinearHeading(new Pose2d(-68, -56, Math.toRadians(0)))
//                            .back(24)
                            .build();
        }

        //blue wings
        {
            OLD_blueWings_StartPositionToDecisionPoint = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(OLD_blueWings_StartPos)
                            .lineTo(new Vector2d(40.5, -64.5), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .forward(6)
                            .turn(Math.toRadians(180))
                            .lineTo(OLD_blueWings_DecisionPointPos.vec(), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();

            OLD_blueWings_DecisionPointToLeftSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(OLD_blueWings_DecisionPointPos)
                            .forward(14)
                            .turn(Math.toRadians(-90))
                            .forward(4)
                            .back(2)
                            .build();

            OLD_blueWings_LeftSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(OLD_blueWings_DecisionPointToLeftSpike.end())
                            .lineToLinearHeading(OLD_blueWings_DecisionPointPos)
                            .build();

            OLD_blueWings_DecisionPointToCenterSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(OLD_blueWings_DecisionPointPos)
                            .forward(3)
                            .build();

            OLD_blueWings_CenterSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(OLD_blueWings_DecisionPointToCenterSpike.end())
                            .lineToLinearHeading(OLD_blueWings_DecisionPointPos)
                            .build();

            OLD_blueWings_DecisionPointToRightSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(OLD_blueWings_DecisionPointPos)
                            .forward(2)
                            .turn(Math.toRadians(35))
                            .forward(10)
                            .back(3)
                            .build();

            OLD_blueWings_RightSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(OLD_blueWings_DecisionPointToRightSpike.end())
                            .lineToLinearHeading(OLD_blueWings_DecisionPointPos)
                            .build();

            OLD_blueWings_DecisionPointToSafetyWaypoint = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(OLD_blueWings_DecisionPointPos)
                            .turn(Math.toRadians(90))
                            .lineToLinearHeading(new Pose2d(-32, -10, Math.toRadians(0))) //38
                            .build();

            OLD_blueWings_SafetyWaypointToBackdropWaypoint = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(OLD_blueWings_DecisionPointToSafetyWaypoint.end())
                            .lineToLinearHeading(OLD_blueWings_BackdropWaypointPos)
                            .build();

            OLD_blueWings_DecisionPointToBackdropWaypoint = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(OLD_blueWings_DecisionPointPos)
                            .lineToLinearHeading(OLD_blueWings_BackdropWaypointPos)
                            .build();

            OLD_blueWings_WaypointToLeftSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(OLD_blueWings_BackdropWaypointPos)
                            .lineToLinearHeading(new Pose2d(-48.625, -40, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))
                            .build();

            OLD_blueWings_WaypointToCenterSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(OLD_blueWings_BackdropWaypointPos)
                            .lineToLinearHeading(new Pose2d(-47.5, -34, Math.toRadians(0)))
                            .build();

            OLD_blueWings_WaypointToRightSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(OLD_blueWings_BackdropWaypointPos)
                            .lineToLinearHeading(new Pose2d(-48.5, -26, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))
                            .build();

            OLD_blueWings_LeftSlotsToBackdropWaypoint =
                    drive.trajectorySequenceBuilder(OLD_blueWings_WaypointToLeftSlots.end())
                            .forward(3)
//                            .lineTo(blueWings_BackdropWaypointPos.vec(), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                    SampleMecanumDrive.getAccelerationConstraint(30))
                            .build();

            OLD_blueWings_CenterSlotsToBackdropWaypoint =
                    drive.trajectorySequenceBuilder(OLD_blueWings_WaypointToCenterSlots.end())
                            .forward(3)
//                            .lineTo(blueWings_BackdropWaypointPos.vec(), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                    SampleMecanumDrive.getAccelerationConstraint(30))
                            .build();

            OLD_blueWings_RightSlotsToBackdropWaypoint =
                    drive.trajectorySequenceBuilder(OLD_blueWings_WaypointToRightSlots.end())
                            .forward(3)
//                            .lineTo(blueWings_BackdropWaypointPos.vec(), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                    SampleMecanumDrive.getAccelerationConstraint(30))
                            .build();

            OLD_blueWings_BackdropToCornerPark = //go to and spin to parking between backdrops
                    drive.trajectorySequenceBuilder(OLD_blueWings_BackdropWaypointPos)
                            .strafeRight(23)
                            .turn(Math.toRadians(-90))
                            .strafeRight(20)
//                            .lineToLinearHeading(new Pose2d(-56, -62, Math.toRadians(0)))
                            .build();

            OLD_blueWings_BackdropToCenterPark =//go to and spin to parking between backdrops
                    drive.trajectorySequenceBuilder(OLD_blueWings_BackdropWaypointPos)
                            .strafeLeft(23)
                            .turn(Math.toRadians(-90))
                            .strafeRight(20)
//                            .lineToLinearHeading(new Pose2d(-52, -8, Math.toRadians(0)))
                            .build();
        }

        //red backstage
        {
            OLD_redBackstage_StartPositionToDecisionPoint = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(OLD_redBackstage_StartPos)
                            .lineTo(new Vector2d(-12.5, 64.5), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .lineTo(OLD_redBackstage_DecisionPointPos.vec(), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();

            OLD_redBackstage_DecisionPointToLeftSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(OLD_redBackstage_DecisionPointPos)
                            .forward(2)
                            .turn(Math.toRadians(45))
                            .forward(14)
                            .back(7)
                            .build();

            OLD_redBackstage_LeftSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(OLD_redBackstage_DecisionPointToLeftSpike.end())
                            .lineToLinearHeading(OLD_redBackstage_DecisionPointPos)
                            .build();

            OLD_redBackstage_DecisionPointToCenterSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(OLD_redBackstage_DecisionPointPos)
                            .forward(18)
                            .back(5)
                            .build();

            OLD_redBackstage_CenterSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(OLD_redBackstage_DecisionPointToCenterSpike.end())
                            .lineToLinearHeading(OLD_redBackstage_DecisionPointPos)
                            .build();

            OLD_redBackstage_DecisionPointToRightSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(OLD_redBackstage_DecisionPointPos)
                            .forward(8)
                            .turn(Math.toRadians(-50))
                            .forward(5)
                            .back(2)
                            .build();

            OLD_redBackstage_RightSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(OLD_redBackstage_DecisionPointToRightSpike.end())
                            .lineToLinearHeading(OLD_redBackstage_DecisionPointPos)
                            .build();

            OLD_redBackstage_DecisionPointToBackdropWaypoint = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(OLD_redBackstage_DecisionPointPos)
                            .lineToLinearHeading(OLD_redBackstage_BackdropWaypointPos)
                            .build();

            OLD_redBackstage_WaypointToLeftSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(OLD_redBackstage_DecisionPointToBackdropWaypoint.end())
                            .lineToLinearHeading(new Pose2d(-50.2, 32, Math.toRadians(0)))
                            .build();

            OLD_redBackstage_WaypointToCenterSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(OLD_redBackstage_DecisionPointToBackdropWaypoint.end())
                            .lineToLinearHeading(new Pose2d(-50, 39, Math.toRadians(0)))
                            .build();

            OLD_redBackstage_WaypointToRightSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(OLD_redBackstage_DecisionPointToBackdropWaypoint.end())
                            .lineToLinearHeading(new Pose2d(-50, 42, Math.toRadians(0)))//46
                            .build();

            OLD_redBackstage_LeftSlotsToBackdropWaypoint =
                    drive.trajectorySequenceBuilder(OLD_redBackstage_DecisionPointToBackdropWaypoint.end())
                            .forward(2)
                            .lineTo(OLD_redBackstage_BackdropWaypointPos.vec(), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(30))
                            .build();

            OLD_redBackstage_CenterSlotsToBackdropWaypoint =
                    drive.trajectorySequenceBuilder(OLD_redBackstage_DecisionPointToBackdropWaypoint.end())
                            .forward(2)
                            .lineTo(OLD_redBackstage_BackdropWaypointPos.vec(), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(30))
                            .build();

            OLD_redBackstage_RightSlotsToBackdropWaypoint =
                    drive.trajectorySequenceBuilder(OLD_redBackstage_DecisionPointToBackdropWaypoint.end())
                            .forward(2)
                            .lineTo(OLD_redBackstage_BackdropWaypointPos.vec(), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(30))
                            .build();

            OLD_redBackstage_WaypointToCornerPark = //go to and spin to parking between backdrops
                    drive.trajectorySequenceBuilder(OLD_redBackstage_BackdropWaypointPos)
                            .strafeLeft(26)
                            .turn(Math.toRadians(90))
                            .strafeLeft(20)
//                            .lineToLinearHeading(new Pose2d(-56, 64, Math.toRadians(0)))
                            .build();

            OLD_redBackstage_WaypointToCenterPark = //go to and spin to parking between backdrops
                    drive.trajectorySequenceBuilder(OLD_redBackstage_BackdropWaypointPos)
                            .strafeRight(26)
                            .turn(Math.toRadians(90))
                            .strafeLeft(20)
//                            .lineToLinearHeading(new Pose2d(-56, 64, Math.toRadians(0)))
                            .build();
        }

        //red wings
        {
            OLD_redWings_StartPositionToDecisionPoint = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(OLD_redWings_StartPos)
                            .lineTo(new Vector2d(40.5, 64.5), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .forward(6)
                            .turn(Math.toRadians(180))
                            .lineTo(OLD_redWings_DecisionPointPos.vec(), SampleMecanumDrive.getVelocityConstraint(86, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();

            OLD_redWings_DecisionPointToLeftSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(OLD_redWings_DecisionPointPos)
                            .forward(12)
                            .turn(Math.toRadians(-45))
                            .forward(12)
                            .back(8)
                            .build();

            OLD_redWings_LeftSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(OLD_redWings_DecisionPointToLeftSpike.end())
                            .lineToLinearHeading(OLD_redWings_DecisionPointPos)
                            .build();

            OLD_redWings_DecisionPointToCenterSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(OLD_redWings_DecisionPointPos)
                            .forward(4)
                            .build();

            OLD_redWings_CenterSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(OLD_redWings_DecisionPointToCenterSpike.end())
                            .lineToLinearHeading(OLD_redWings_DecisionPointPos)
                            .build();

            OLD_redWings_DecisionPointToRightSpike = //shift and go forward to score center
                    drive.trajectorySequenceBuilder(OLD_redWings_DecisionPointPos)
                            .forward(18)
                            .turn(Math.toRadians(90))
                            .forward(4)
                            .build();

            OLD_redWings_RightSpikeToDecisionPoint =  //Shift to decision point
                    drive.trajectorySequenceBuilder(OLD_redWings_DecisionPointToRightSpike.end())
                            .lineToLinearHeading(OLD_redWings_DecisionPointPos)
                            .build();

            OLD_redWings_DecisionPointToSafetyWaypoint = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(OLD_redWings_DecisionPointPos)
                            .turn(Math.toRadians(-90))
                            .lineToLinearHeading(new Pose2d(-38, 14, Math.toRadians(0)))
                            .build();

            OLD_redWings_SafetyWaypointToBackdropWaypoint = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(OLD_redWings_DecisionPointToSafetyWaypoint.end())
                            .lineToLinearHeading(OLD_redWings_BackdropWaypointPos)
                            .build();

            OLD_redWings_DecisionPointToBackdropWaypoint = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(OLD_redWings_DecisionPointPos)
                            .lineToLinearHeading(OLD_redWings_BackdropWaypointPos)
                            .build();

            OLD_redWings_WaypointToLeftSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(OLD_redWings_SafetyWaypointToBackdropWaypoint.end())
                            .lineToLinearHeading(new Pose2d(-51, 33, Math.toRadians(0)))
                            .build();

            OLD_redWings_WaypointToCenterSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(OLD_redWings_SafetyWaypointToBackdropWaypoint.end())
                            .lineToLinearHeading(new Pose2d(-49.5, 41, Math.toRadians(0)))
                            .build();

            OLD_redWings_WaypointToRightSlots = //decision point to position around backdrop to prep for where to score yellow
                    drive.trajectorySequenceBuilder(OLD_redWings_SafetyWaypointToBackdropWaypoint.end())
                            .lineToLinearHeading(new Pose2d(-50, 46, Math.toRadians(0)))
                            .build();

            OLD_redWings_LeftSlotsToBackdropWaypoint =
                    drive.trajectorySequenceBuilder(OLD_redWings_DecisionPointToBackdropWaypoint.end())
                            .forward(3)
//                            .lineTo(redWings_BackdropWaypointPos.vec(), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                    SampleMecanumDrive.getAccelerationConstraint(30))
                            .build();

            OLD_redWings_CenterSlotsToBackdropWaypoint =
                    drive.trajectorySequenceBuilder(OLD_redWings_DecisionPointToBackdropWaypoint.end())
                            .forward(3)
//                            .lineTo(redWings_BackdropWaypointPos.vec(), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                    SampleMecanumDrive.getAccelerationConstraint(30))
                            .build();

            OLD_redWings_RightSlotsToBackdropWaypoint =
                    drive.trajectorySequenceBuilder(OLD_redWings_DecisionPointToBackdropWaypoint.end())
                            .forward(3)
//                            .lineTo(redWings_BackdropWaypointPos.vec(), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                    SampleMecanumDrive.getAccelerationConstraint(30))
                            .build();

            OLD_redWings_BackdropToCenterPark = //go to and spin to parking between backdrops
                    drive.trajectorySequenceBuilder(OLD_redWings_BackdropWaypointPos)
//                            .forward(4)
                            .strafeRight(26)
                            .turn(Math.toRadians(90))
                            .strafeLeft(20)
//                            .back(12)
//                            .lineToLinearHeading(new Pose2d(-56, 62, Math.toRadians(0)))
                            .build();

            OLD_redWings_BackdropToCornerPark = //go to and spin to parking between backdrops
                    drive.trajectorySequenceBuilder(OLD_redWings_BackdropWaypointPos)
                            .strafeLeft(28)
                            .turn(Math.toRadians(90))
                            .strafeLeft(20)
////                            .back(12)
//                            .lineToLinearHeading(new Pose2d(-56, 62, Math.toRadians(0)))
                            .build();
        }

    }
}
