package org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos;


import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_SpikeToBackdrop;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_StartToCenterSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_StartToLeftSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_StartToRightSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_StackPickupSequence;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_StartToCenterSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_StartToLeftSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_StartToRightSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_CenterSpikeToStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_LeftSpikeToStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_RightSpikeToStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_TransitToBackstageViaDoor;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_TransitToBackstageViaDoorWait;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_TransitToBackstageViaTruss;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_TransitToBackstageViaTrussWait;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropToStackViaDoor;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropToStackViaTruss;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackstageToStackViaDoor;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackstageToStackViaTruss;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.accelConstraint40in;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.redWings_StartPos;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.velConstraint10in;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.velConstraint20in;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.ARTEMIS.auto.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.autoCommands.AutoBackdropDepositCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.intakeCommands.IntakeInCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.intakeCommands.IntakeOutCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.LiftToPositionCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.RobotAlignToTagRange;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.RobotToStateCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Arm;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Intake;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.LEDs;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Lift;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Webcams;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Winch;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.ARTEMIS.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.ARTEMIS.visionTesting.BluePropDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

import java.util.concurrent.TimeUnit;

//@Disabled
@Autonomous(group = "drive", name = "\uDFE5 RedAuto")
public class RedAuto extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private SampleMecanumDrive drive;
    private VisionPortal portal;
    private BluePropDetection bluePropThreshold;
    private static AprilTagLibrary tags = AprilTagGameDatabase.getCurrentGameTagLibrary();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private LEDs leds;
    private Gripper gripper;
    private Wrist wrist;
    private Arm arm;
    private Lift lift;
    private Intake intake;
    private Winch winch;
    private Drivetrain drivetrain;
    private Webcams webcam;

    private Gamepad currentGamepad, previousGamepad;

    private String startingSide, cycleTarget, transitVia, parkIn;
    private boolean cycle = false, wait = false;

    private static RobotToStateCommand robotToDropPurple, robotToDeposit, robotToIntake;

    private static TrajectorySequence StartToSpike, SpikeToStack, StackToBack, BackToStack;

    private static RobotAlignToTagRange robotAlignToLeftTag, robotAlignToCenterTag, robotAlignToRightTag;

    private static AprilTagMetadata targetBackdropTag;


    @Override
    public void initialize() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);
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


//        drive.setPoseEstimate();
        EXCCMP_AutoTrajectories.generateTrajectories(drive);

        intake = new Intake(hardwareMap);
        leds = new LEDs(hardwareMap);
        lift = new Lift(hardwareMap);
        gripper = new Gripper(hardwareMap);
        wrist = new Wrist(hardwareMap);
        arm = new Arm(hardwareMap);
        winch = new Winch(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        webcam = new Webcams(hardwareMap);

        robotToDropPurple = new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "dropPurple");
        robotToDeposit = new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "deposit");
        robotToIntake = new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "intake");

        gripper.grabRight();
        intake.up();

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            // determine target backdrop tag
            if (bluePropThreshold.getPropPosition() == "left")
                targetBackdropTag = tags.lookupTag(4);
            else if (bluePropThreshold.getPropPosition() == "right")
                targetBackdropTag = tags.lookupTag(6);
            else
                targetBackdropTag = tags.lookupTag(5);
            telemetry.addLine("target backdrop tag: " + targetBackdropTag.name);

            telemetry.addData("Press DPAD LEFT ⬅️ for WINGS and DPAD RIGHT ➡️ for BACKSTAGE ", startingSide);
            if (currentGamepad.dpad_left)
                startingSide = "wing";
            if (currentGamepad.dpad_right)
                startingSide = "backstage";

            telemetry.addData("Press DPAD UP to change cycle: ", cycle);
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up)
                cycle = !cycle;

            if (!cycle) {
                telemetry.addData("Press X to change park target: ", parkIn);
                if (currentGamepad.dpad_down && !previousGamepad.dpad_down)
                    if (cycleTarget != "center")
                        cycleTarget = "center";
                    else
                        cycleTarget = "corner";
            }

            telemetry.addData("Press B to change wait: ", wait);
            if (currentGamepad.b && !previousGamepad.b)
                wait = !wait;

            telemetry.addData("TRANSIT: Y for DOOR or A for TRUSS", transitVia);
            if (currentGamepad.y) {
                transitVia = "door";
            }
            if (currentGamepad.a) {
                transitVia = "truss";
            }

            telemetry.addData("Press DPAD_DOWN ⬇️ to change cycle target: ", cycleTarget);
            if (currentGamepad.dpad_down && !previousGamepad.dpad_down)
                if (cycleTarget != "backstage")
                    cycleTarget = "backstage";
                else
                    cycleTarget = "backdrop";

            //determine trajectories
            if (startingSide.toLowerCase() == "wing") {
                drive.setPoseEstimate(redWings_StartPos);

                //determine wing trajectories
                if (bluePropThreshold.getPropPosition() == "left") {
                    StartToSpike = RedWings_StartToLeftSpike;
                    SpikeToStack = RedWings_LeftSpikeToStack;

                    telemetry.addLine("left spike traj");
                } else if (bluePropThreshold.getPropPosition() == "right") {
                    StartToSpike = RedWings_StartToRightSpike;
                    SpikeToStack = RedWings_RightSpikeToStack;

                    telemetry.addLine("right spike traj");
                } else {
                    StartToSpike = RedWings_StartToCenterSpike;
                    StartToSpike = RedWings_CenterSpikeToStack;

                    telemetry.addLine("center spike traj");
                }
            } else {
                if (bluePropThreshold.getPropPosition() == "left") {
                    StartToSpike = RedBackstage_StartToLeftSpike;
                    telemetry.addLine("left spike traj");
                } else if (bluePropThreshold.getPropPosition() == "right") {
                    StartToSpike = RedBackstage_StartToRightSpike;
                    telemetry.addLine("right spike traj");
                } else {
                    StartToSpike = RedBackstage_StartToCenterSpike;
                    telemetry.addLine("center spike traj");
                }
            }
            if (transitVia == "door") {
                if (wait)
                    StackToBack = RedWings_TransitToBackstageViaDoorWait;
                else
                    StackToBack = RedWings_TransitToBackstageViaDoor;

                if (cycleTarget == "backdrop")
                    BackToStack = Red_BackdropToStackViaDoor;
                else
                    BackToStack = Red_BackstageToStackViaDoor;
            } else { // transiting via truss
                if (wait)
                    StackToBack = RedWings_TransitToBackstageViaTrussWait;
                else
                    StackToBack = RedWings_TransitToBackstageViaTruss;

                if (cycleTarget == "backstage")
                    BackToStack = Red_BackstageToStackViaTruss;
                else
                    BackToStack = Red_BackdropToStackViaTruss;
            }

            telemetry.addLine("waitForStart");
            telemetry.addData("Prop Position", bluePropThreshold.getPropPosition());
            telemetry.update();
            sleep(20);
        }

        telemetry.update();
        // drop purple
        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> leds.setLEDstate("purple")),
                new ParallelCommandGroup(
                        new FollowTrajectoryCommand(drive, StartToSpike),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                robotToDropPurple
                        )
                ),
                new WaitCommand(100),
                new InstantCommand(gripper::releaseLeft)
        ));

        // get extra pixel on first cycle (start: spike, end: back)
        if (startingSide == "wing") {
            schedule(new SequentialCommandGroup(
                    new InstantCommand(() -> leds.setLEDstate("white")),
                    new ParallelCommandGroup(
                            new FollowTrajectoryCommand(drive, SpikeToStack),
                            new SequentialCommandGroup(
                                    new WaitCommand(200),
                                    robotToIntake
                            )
                    ),
                    new WaitCommand(300),
                    new ParallelCommandGroup(
                            new IntakeInCommand(intake, leds),
                            new FollowTrajectoryCommand(drive, RedWings_StackPickupSequence)
                    ),
                    new ParallelCommandGroup(
                            new InstantCommand(() -> leds.setLEDstate("yellow")),
                            new SequentialCommandGroup(
                                    new InstantCommand(gripper::grabLeft),
                                    new InstantCommand(gripper::grabRight),
                                    new WaitCommand(200),
                                    new IntakeOutCommand(intake)
                            ),
                            new FollowTrajectoryCommand(drive, StackToBack)
                    )
            ));
        } else {
            schedule(new SequentialCommandGroup( // turn bot around to align with backdrop
                    new FollowTrajectoryCommand(drive, RedBackstage_SpikeToBackdrop)
            ));
        }

        // align to atag and deposit sequence
        schedule(new SequentialCommandGroup(
                new AutoBackdropDepositCommand(arm, wrist, gripper, lift, intake, winch, leds, drivetrain, webcam, drive, targetBackdropTag)
        ));

        if (cycle) {
            schedule(new SequentialCommandGroup(
                    // back to intake to back AutoBackToStackToBackCommand
                    new InstantCommand(() -> leds.setLEDstate("plaid")),
                    new FollowTrajectoryCommand(drive, BackToStack),
                    // intake in sequence
                    new ParallelCommandGroup(
                            new IntakeInCommand(intake, leds),
                            new FollowTrajectoryCommand(drive, RedWings_StackPickupSequence)
                    ),
                    new ParallelCommandGroup(
                            new InstantCommand(() -> leds.setLEDstate("yellow")),
                            new SequentialCommandGroup(
                                    new InstantCommand(gripper::grabLeft),
                                    new InstantCommand(gripper::grabRight),
                                    new WaitCommand(200),
                                    new IntakeOutCommand(intake)
                            ),
                            new FollowTrajectoryCommand(drive, StackToBack)
                    )

            ));

            // deposit dequence
            if (cycleTarget == "backdrop") {
                schedule(new SequentialCommandGroup(
                        new AutoBackdropDepositCommand(arm, wrist, gripper, lift, intake, winch, leds, drivetrain, webcam, drive, targetBackdropTag)
                ));
            } else {
                schedule(new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> leds.setLEDstate("white")),
                                new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "dropPurple")
                        ),
                        new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(drive.getPoseEstimate()).setReversed(true).forward(2, velConstraint10in, accelConstraint40in).build()),
                        new InstantCommand(gripper::releaseRight),
                        new InstantCommand(gripper::releaseLeft),
                        new WaitCommand(100),
                        new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(drive.getPoseEstimate()).setReversed(true).back(2, velConstraint10in, accelConstraint40in).build()),
                        new InstantCommand(() -> leds.setLEDstate("purple")),
                        new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "intake")
                ));
            }
            // cycle code
            // park in middle code
        } else {
            //park code
            if (parkIn == "corner") {
                schedule(new SequentialCommandGroup(
                        new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(drive.getPoseEstimate()).setReversed(true).strafeRight(24, velConstraint20in, accelConstraint40in).build()),
                        new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(drive.getPoseEstimate()).setReversed(true).back(24, velConstraint20in, accelConstraint40in).build())
                ));
            } else {
                schedule(new SequentialCommandGroup(
                        new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(drive.getPoseEstimate()).setReversed(true).strafeLeft(24, velConstraint20in, accelConstraint40in).build()),
                        new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(drive.getPoseEstimate()).setReversed(true).back(24, velConstraint20in, accelConstraint40in).build())
                ));
            }
        }
    }
}