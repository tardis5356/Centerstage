package org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos;

import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueWings_BackdropToCornerPark;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueWings_CenterSlotsToBackdropWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueWings_CenterSpikeToDecisionPoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueWings_DecisionPointToCenterSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueWings_DecisionPointToLeftSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueWings_DecisionPointToRightSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueWings_DecisionPointToSafetyWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueWings_LeftSlotsToBackdropWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueWings_LeftSpikeToDecisionPoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueWings_RightSlotsToBackdropWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueWings_RightSpikeToDecisionPoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueWings_SafetyWaypointToBackdropWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueWings_StartPos;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueWings_StartPositionToDecisionPoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueWings_WaypointToCenterSlots;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueWings_WaypointToLeftSlots;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueWings_WaypointToRightSlots;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.ARTEMIS.auto.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.RobotToStateCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Arm;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Intake;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.LEDs;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Lift;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Winch;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.ARTEMIS.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.ARTEMIS.visionTesting.BluePropDetection;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

//public class CSTB_blueWings_Park {
//@Disabled
@Autonomous(group = "drive", name = "blueWings FARBACK Purple+Yellow+CornerPark")
public class blueWings_FARBACK_PurpleYellowCornerParkAuto extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private SampleMecanumDrive drive;
    private VisionPortal portal;
    private BluePropDetection bluePropThreshold;
    //    private Lift lift;
    public static TrajectorySequence blueWings_DecisionPointToSpike, blueWings_SpikeToDecisionPoint, blueWings_WaypointToBackdrop, blueWings_BackdropToWaypoint;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private LEDs leds;
    private Gripper gripper;
    private Wrist wrist;
    private Arm arm;
    private Lift lift;
    private Intake intake;
    private Winch winch;

    @Override
    public void initialize() {
//        MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ////////‼️‼️⁉️⁉️CAMERA INITIALIZATION/DEFINING ⁉️⁉️⁉️
//        public void runOpMode () throw InterruptedException {
        bluePropThreshold = new BluePropDetection();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(bluePropThreshold)
                .setCameraResolution(new Size(640, 480))
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .enableLiveView(true)
//                .setCamera()
                .addProcessor(bluePropThreshold)
                .build();

        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
        GainControl gainControl = portal.getCameraControl(GainControl.class);

        boolean wasExposureSet = exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(50, TimeUnit.MILLISECONDS);
        gainControl.setGain(0);

//      everything above is included in the previous comment, this order needs to be maintained.

//            waitForStart();
//        }

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(blueWings_StartPos);
        Artemis_PurpleYellowParkAutoTrajectories.generateTrajectories(drive);

        intake = new Intake(hardwareMap);
        leds = new LEDs(hardwareMap);
        lift = new Lift(hardwareMap);
        gripper = new Gripper(hardwareMap);
        wrist = new Wrist(hardwareMap);
        arm = new Arm(hardwareMap);
        winch = new Winch(hardwareMap);

        gripper.grabRight();
        intake.up();
////////////////////////////DEFINING PARK TRAJECTORIES//////////////////////////////

        ////////////////////////////////////DONE DEFINING PARK TRAJECTORIES///////////////////////////////////////

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            switch (bluePropThreshold.getPropPosition()) {
                case "left":
                    blueWings_DecisionPointToSpike = blueWings_DecisionPointToLeftSpike;
                    blueWings_SpikeToDecisionPoint = blueWings_LeftSpikeToDecisionPoint;

                    blueWings_WaypointToBackdrop = blueWings_WaypointToLeftSlots;
                    blueWings_BackdropToWaypoint = blueWings_LeftSlotsToBackdropWaypoint;

                    telemetry.addLine("park traj 1");
                    break;

                default:
                case "center":
                    blueWings_DecisionPointToSpike = blueWings_DecisionPointToCenterSpike;
                    blueWings_SpikeToDecisionPoint = blueWings_CenterSpikeToDecisionPoint;

                    blueWings_WaypointToBackdrop = blueWings_WaypointToCenterSlots;
                    blueWings_BackdropToWaypoint = blueWings_CenterSlotsToBackdropWaypoint;

                    telemetry.addLine("park traj 2");
                    break;
                case "right":
                    blueWings_DecisionPointToSpike = blueWings_DecisionPointToRightSpike;
                    blueWings_SpikeToDecisionPoint = blueWings_RightSpikeToDecisionPoint;

                    blueWings_WaypointToBackdrop = blueWings_WaypointToRightSlots;
                    blueWings_BackdropToWaypoint = blueWings_RightSlotsToBackdropWaypoint;

                    telemetry.addLine("park traj 3");
                    break;
            }

            telemetry.addLine("waitForStart");
            telemetry.addData("Prop Position", bluePropThreshold.getPropPosition());
            telemetry.update();
            sleep(20);
        }

        telemetry.update();
        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> leds.setLEDstate("purple")),
                new FollowTrajectoryCommand(drive, blueWings_StartPositionToDecisionPoint),
                new FollowTrajectoryCommand(drive, blueWings_DecisionPointToSpike),
                new InstantCommand(intake::slowOut),
                new WaitCommand(500),
                new InstantCommand(intake::stop),
                new FollowTrajectoryCommand(drive, blueWings_SpikeToDecisionPoint),
                new InstantCommand(() -> leds.setLEDstate("yellow")),
                new WaitCommand(6500),
                new FollowTrajectoryCommand(drive, blueWings_DecisionPointToSafetyWaypoint),
                new FollowTrajectoryCommand(drive, blueWings_SafetyWaypointToBackdropWaypoint),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(750),
                                new FollowTrajectoryCommand(drive, blueWings_WaypointToBackdrop)
                        ),
                        new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "deposit")
                ),
                new WaitCommand(250),
                new InstantCommand(gripper::releaseRight),
                new WaitCommand(500),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowTrajectoryCommand(drive, blueWings_BackdropToWaypoint),
                                new InstantCommand(() -> leds.setLEDstate("idle"))//,
//                                new FollowTrajectoryCommand(drive, blueWings_BackdropToCornerPark)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "intake")
                        )
                )


        ));

    }
}
