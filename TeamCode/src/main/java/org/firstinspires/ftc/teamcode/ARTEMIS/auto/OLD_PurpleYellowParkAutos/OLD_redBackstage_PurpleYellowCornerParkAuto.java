package org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleYellowParkAutos;

import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleYellowParkAutos.OLD_Artemis_PurpleYellowParkAutoTrajectories.OLD_redBackstage_CenterSlotsToBackdropWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleYellowParkAutos.OLD_Artemis_PurpleYellowParkAutoTrajectories.OLD_redBackstage_CenterSpikeToDecisionPoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleYellowParkAutos.OLD_Artemis_PurpleYellowParkAutoTrajectories.OLD_redBackstage_DecisionPointToBackdropWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleYellowParkAutos.OLD_Artemis_PurpleYellowParkAutoTrajectories.OLD_redBackstage_DecisionPointToCenterSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleYellowParkAutos.OLD_Artemis_PurpleYellowParkAutoTrajectories.OLD_redBackstage_DecisionPointToLeftSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleYellowParkAutos.OLD_Artemis_PurpleYellowParkAutoTrajectories.OLD_redBackstage_DecisionPointToRightSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleYellowParkAutos.OLD_Artemis_PurpleYellowParkAutoTrajectories.OLD_redBackstage_LeftSlotsToBackdropWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleYellowParkAutos.OLD_Artemis_PurpleYellowParkAutoTrajectories.OLD_redBackstage_LeftSpikeToDecisionPoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleYellowParkAutos.OLD_Artemis_PurpleYellowParkAutoTrajectories.OLD_redBackstage_RightSlotsToBackdropWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleYellowParkAutos.OLD_Artemis_PurpleYellowParkAutoTrajectories.OLD_redBackstage_RightSpikeToDecisionPoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleYellowParkAutos.OLD_Artemis_PurpleYellowParkAutoTrajectories.OLD_redBackstage_StartPos;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleYellowParkAutos.OLD_Artemis_PurpleYellowParkAutoTrajectories.OLD_redBackstage_StartPositionToDecisionPoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleYellowParkAutos.OLD_Artemis_PurpleYellowParkAutoTrajectories.OLD_redBackstage_WaypointToCenterSlots;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleYellowParkAutos.OLD_Artemis_PurpleYellowParkAutoTrajectories.OLD_redBackstage_WaypointToCornerPark;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleYellowParkAutos.OLD_Artemis_PurpleYellowParkAutoTrajectories.OLD_redBackstage_WaypointToLeftSlots;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleYellowParkAutos.OLD_Artemis_PurpleYellowParkAutoTrajectories.OLD_redBackstage_WaypointToRightSlots;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.ARTEMIS.vision.RedPropDetection;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

//public class CSTB_redWings_Park {
@Disabled
@Autonomous(group = "drive", name = "redBackstage Purple+Yellow+CornerPark")
public class OLD_redBackstage_PurpleYellowCornerParkAuto extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private SampleMecanumDrive drive;
    private VisionPortal portal;
    private RedPropDetection redPropThreshold;
    //    private Lift lift;
    private static TrajectorySequence redBackstage_DecisionPointToSpike, redBackstage_SpikeToDecisionPoint, redBackstage_WaypointToBackdrop, redBackstage_BackdropToWaypoint;
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
        redPropThreshold = new RedPropDetection();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(redPropThreshold)
                .setCameraResolution(new Size(640, 480))
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .enableLiveView(true)
//                .setCamera()
                .addProcessor(redPropThreshold)
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

        drive.setPoseEstimate(OLD_redBackstage_StartPos);
        OLD_Artemis_PurpleYellowParkAutoTrajectories.generateTrajectories(drive);

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
            switch (redPropThreshold.getPropPosition()) {
                case "left":
                    redBackstage_DecisionPointToSpike = OLD_redBackstage_DecisionPointToLeftSpike;
                    redBackstage_SpikeToDecisionPoint = OLD_redBackstage_LeftSpikeToDecisionPoint;

                    redBackstage_WaypointToBackdrop = OLD_redBackstage_WaypointToLeftSlots;
                    redBackstage_BackdropToWaypoint = OLD_redBackstage_LeftSlotsToBackdropWaypoint;

                    telemetry.addLine("park traj 1");
                    break;
                default:
                case "center":
                    redBackstage_DecisionPointToSpike = OLD_redBackstage_DecisionPointToCenterSpike;
                    redBackstage_SpikeToDecisionPoint = OLD_redBackstage_CenterSpikeToDecisionPoint;

                    redBackstage_WaypointToBackdrop = OLD_redBackstage_WaypointToCenterSlots;
                    redBackstage_BackdropToWaypoint = OLD_redBackstage_CenterSlotsToBackdropWaypoint;

                    telemetry.addLine("park traj 2");
                    break;
                case "right":
                    redBackstage_DecisionPointToSpike = OLD_redBackstage_DecisionPointToRightSpike;
                    redBackstage_SpikeToDecisionPoint = OLD_redBackstage_RightSpikeToDecisionPoint;

                    redBackstage_WaypointToBackdrop = OLD_redBackstage_WaypointToRightSlots;
                    redBackstage_BackdropToWaypoint = OLD_redBackstage_RightSlotsToBackdropWaypoint;

                    telemetry.addLine("park traj 3");
                    break;
            }

            telemetry.addLine("waitForStart");
            telemetry.addData("Prop Position", redPropThreshold.getPropPosition());
            telemetry.update();
            sleep(20);
        }


        telemetry.update();
        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> leds.setLEDstate("purple")),
                new FollowTrajectoryCommand(drive, OLD_redBackstage_StartPositionToDecisionPoint),
                new FollowTrajectoryCommand(drive, redBackstage_DecisionPointToSpike),
                new InstantCommand(intake::superSlowOut),
                new WaitCommand(500),
                new InstantCommand(intake::stop),
                new FollowTrajectoryCommand(drive, redBackstage_SpikeToDecisionPoint),
                new InstantCommand(() -> leds.setLEDstate("yellow")),
                new FollowTrajectoryCommand(drive, OLD_redBackstage_DecisionPointToBackdropWaypoint),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(750),
                                new FollowTrajectoryCommand(drive, redBackstage_WaypointToBackdrop)
                        ),
                        new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "depositHigh")
                ),
                new WaitCommand(250),
                new InstantCommand(gripper::releaseRight),
                new WaitCommand(500),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowTrajectoryCommand(drive, redBackstage_BackdropToWaypoint),
                                new InstantCommand(() -> leds.setLEDstate("idle")),
                                new FollowTrajectoryCommand(drive, OLD_redBackstage_WaypointToCornerPark)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "intake")
                        )
                )


        ));

    }
}
