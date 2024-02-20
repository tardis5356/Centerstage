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
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_TransitToBackdropViaDoor;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_TransitToBackdropViaDoorWait;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_TransitToBackdropViaTruss;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_TransitToBackdropViaTrussWait;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_TransitToBackstageViaDoor;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_TransitToBackstageViaDoorWait;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_TransitToBackstageViaTruss;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_TransitToBackstageViaTrussWait;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropToStackViaDoor;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropToStackViaTruss;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackstageToStackViaDoor;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackstageToStackViaTruss;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.redWings_StartPos;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
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
@Autonomous(group = "drive", name = "\uD83D\uDFE5 RedAuto") //
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

    private String startingSide = "wing", cycleTarget = "backdrop", transitVia = "door", parkIn = "center";
    private boolean deliverYellow = true, cycle = false, wait = false;

    private static RobotToStateCommand robotToDropPurple, robotToDeposit, robotToIntake;

    private static TrajectorySequence StartToSpike, SpikeToStack, StackToBack, BackToStack, SpikeToBackdrop, StackPickupSequence;

    private static RobotAlignToTagRange robotAlignToLeftTag, robotAlignToCenterTag, robotAlignToRightTag;

    private static AprilTagMetadata targetBackdropTag;

    private boolean commandsScheduled = false;

    AutoGenerator autoGenerator;
    private SequentialCommandGroup autoCommands;

    @Override
    public void initialize() {
        currentGamepad = new Gamepad();
        previousGamepad = new Gamepad();

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

        autoCommands = new SequentialCommandGroup();
        autoGenerator = new AutoGenerator();

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

        gripper.grabRight();
        gripper.grabLeft();
        intake.up();

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // determine target backdrop tag
            if (bluePropThreshold.getPropPosition() == "left")
                targetBackdropTag = tags.lookupTag(4);
            else if (bluePropThreshold.getPropPosition() == "right")
                targetBackdropTag = tags.lookupTag(6);
            else
                targetBackdropTag = tags.lookupTag(5);
            telemetry.addLine("target backdrop tag: " + targetBackdropTag.name);

            // red blue 🔵🟥

            //

            telemetry.addData("⬅️ LEFT | startingSide: ", startingSide);
            if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
                if (startingSide != "wing")
                    startingSide = "wing";
                else
                    startingSide = "backstage";
            }

            telemetry.addData("UP | cycle y/n: ", cycle);
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up)
                cycle = !cycle;

            if (!cycle) {
                telemetry.addData("X | park target: ", parkIn);
                if (currentGamepad.x && !previousGamepad.x)
                    if (parkIn != "center")
                        parkIn = "center";
                    else
                        parkIn = "corner";
            }

            telemetry.addData("B | wait: ", wait);
            if (currentGamepad.b && !previousGamepad.b)
                wait = !wait;

            telemetry.addData("A | deliverYellow: ", deliverYellow);
            if (currentGamepad.a && !previousGamepad.a)
                deliverYellow = !deliverYellow;

            telemetry.addData("Y | TRANSIT: ", transitVia);
            if (currentGamepad.y && !previousGamepad.y) {
                if (transitVia != "door")
                    transitVia = "door";
                else
                    transitVia = "truss";
            }

            telemetry.addData("⬇️ DOWN | CYCLE TARGET: ", cycleTarget);
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
                    SpikeToStack = RedWings_CenterSpikeToStack;

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
                    if (deliverYellow)
                        StackToBack = RedWings_TransitToBackdropViaDoorWait;
                    else
                        StackToBack = RedWings_TransitToBackstageViaDoorWait;
                else
                    if(deliverYellow)
                        StackToBack = RedWings_TransitToBackdropViaDoor;
                    else
                        StackToBack = RedWings_TransitToBackstageViaDoor;

                if (cycleTarget == "backdrop")
                    BackToStack = Red_BackdropToStackViaDoor;
                else
                    BackToStack = Red_BackstageToStackViaDoor;
            } else { // transiting via truss
                if (wait)
                    if(deliverYellow)
                        StackToBack = RedWings_TransitToBackdropViaTrussWait;
                    else
                        StackToBack = RedWings_TransitToBackstageViaTrussWait;
                else
                    if(deliverYellow)
                        StackToBack = RedWings_TransitToBackdropViaTruss;
                    else
                        StackToBack = RedWings_TransitToBackstageViaTruss;

                if (cycleTarget == "backstage")
                    BackToStack = Red_BackstageToStackViaTruss;
                else
                    BackToStack = Red_BackdropToStackViaTruss;
            }

            StackPickupSequence = RedWings_StackPickupSequence;
            SpikeToBackdrop = RedBackstage_SpikeToBackdrop;

            telemetry.addLine("waitForStart");
            telemetry.addData("Prop Position", bluePropThreshold.getPropPosition());
            telemetry.update();
            sleep(20);
        }

        if(isStopRequested()){
            portal.close();
            return;
        }

        if (isStarted() && !commandsScheduled) {
            // drop purple
            /*
            GenerateAutoCommands passes these parameters

            Arm arm, Wrist wrist, Gripper gripper, Lift lift, Intake intake, Winch winch, LEDs leds,
            Drivetrain drivetrain, Webcams webcam, AprilTagMetadata targetBackdropTag,
            SampleMecanumDrive drive,
            TrajectorySequence StartToSpike, TrajectorySequence SpikeToStack, TrajectorySequence StackToBack,
            TrajectorySequence BackToStack, TrajectorySequence SpikeToBackdrop, TrajectorySequence StackPickupSequence,
            String startingSide, String cycleTarget, String transitVia, String parkIn, boolean cycle, boolean wait

            */

            schedule(
                    autoGenerator.generateAutoCommands(
                            arm, wrist, gripper, lift, intake, winch, leds, drivetrain, webcam,
                            targetBackdropTag, webcam.getActiveAprilTagProcessor(),
                            drive, StartToSpike, SpikeToStack, StackToBack, BackToStack, SpikeToBackdrop, StackPickupSequence,
                            "red", startingSide, cycleTarget, transitVia, parkIn, cycle, wait, deliverYellow
                    )
            );
//            portal.stopLiveView();
//            portal.stopStreaming();
            portal.close();
            commandsScheduled = true;
//            CommandScheduler.getInstance().run();
        }

        while(opModeIsActive()){
            CommandScheduler.getInstance().run();
            telemetry.addData("liftBase", lift.getLiftBase());
            telemetry.addData("liftBase", drive.getPoseEstimate());
            telemetry.update();
        }

        telemetry.update();
    }
}