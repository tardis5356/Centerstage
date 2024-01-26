package org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos;

import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueBackstage_BackdropToCornerPark;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueBackstage_CenterSlotsToBackdropWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueBackstage_CenterSpikeToDecisionPoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueBackstage_DecisionPointToBackdropWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueBackstage_DecisionPointToCenterSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueBackstage_DecisionPointToLeftSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueBackstage_DecisionPointToRightSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueBackstage_LeftSlotsToBackdropWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueBackstage_LeftSpikeToDecisionPoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueBackstage_RightSlotsToBackdropWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueBackstage_RightSpikeToDecisionPoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueBackstage_StartPos;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueBackstage_StartPositionToDecisionPoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueBackstage_WaypointToCenterSlots;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueBackstage_WaypointToLeftSlots;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.PurpleYellowParkAutos.Artemis_PurpleYellowParkAutoTrajectories.blueBackstage_WaypointToRightSlots;

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
@Autonomous(group = "drive", name = "blueBackstage Purple+Yellow+CornerPark")
public class blueBackstage_PurpleYellowCornerParkAuto extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private SampleMecanumDrive drive;
    private VisionPortal portal;
    private BluePropDetection bluePropThreshold;
    //    private Lift lift;
    public static TrajectorySequence blueBackstage_DecisionPointToSpike, blueBackstage_SpikeToDecisionPoint, blueBackstage_WaypointToBackdrop, blueBackstage_BackdropToBackdropWaypoint;
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

        drive.setPoseEstimate(blueBackstage_StartPos);
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
                    blueBackstage_DecisionPointToSpike = blueBackstage_DecisionPointToLeftSpike;
                    blueBackstage_SpikeToDecisionPoint = blueBackstage_LeftSpikeToDecisionPoint;

                    blueBackstage_WaypointToBackdrop = blueBackstage_WaypointToLeftSlots;
                    blueBackstage_BackdropToBackdropWaypoint = blueBackstage_LeftSlotsToBackdropWaypoint;

                    telemetry.addLine("park traj 1");
                    break;
                default:
                case "center":
                    blueBackstage_DecisionPointToSpike = blueBackstage_DecisionPointToCenterSpike;
                    blueBackstage_SpikeToDecisionPoint = blueBackstage_CenterSpikeToDecisionPoint;

                    blueBackstage_WaypointToBackdrop = blueBackstage_WaypointToCenterSlots;
                    blueBackstage_BackdropToBackdropWaypoint = blueBackstage_CenterSlotsToBackdropWaypoint;

                    telemetry.addLine("park traj 2");
                    break;
                case "right":
                    blueBackstage_DecisionPointToSpike = blueBackstage_DecisionPointToRightSpike;
                    blueBackstage_SpikeToDecisionPoint = blueBackstage_RightSpikeToDecisionPoint;

                    blueBackstage_WaypointToBackdrop = blueBackstage_WaypointToRightSlots;
                    blueBackstage_BackdropToBackdropWaypoint = blueBackstage_RightSlotsToBackdropWaypoint;

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
                new FollowTrajectoryCommand(drive, blueBackstage_StartPositionToDecisionPoint),
                new FollowTrajectoryCommand(drive, blueBackstage_DecisionPointToSpike),
                new InstantCommand(intake::slowOut),
                new WaitCommand(750),
                new InstantCommand(intake::stop),
                new FollowTrajectoryCommand(drive, blueBackstage_SpikeToDecisionPoint),
                new InstantCommand(() -> leds.setLEDstate("yellow")),
                new FollowTrajectoryCommand(drive, blueBackstage_DecisionPointToBackdropWaypoint),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(750),
                                new FollowTrajectoryCommand(drive, blueBackstage_WaypointToBackdrop)
                        ),
                        new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "deposit")
                ),
                new WaitCommand(250),
                new InstantCommand(gripper::releaseRight),
                new WaitCommand(500),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowTrajectoryCommand(drive, blueBackstage_BackdropToBackdropWaypoint),
                                new InstantCommand(() -> leds.setLEDstate("idle")),
                                new FollowTrajectoryCommand(drive, blueBackstage_BackdropToCornerPark)
                                ),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "intake")
                        )
                )


        ));

    }
}
