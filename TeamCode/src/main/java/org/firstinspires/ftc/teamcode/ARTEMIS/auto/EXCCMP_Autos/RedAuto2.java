package org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos;


import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_BackdropRelocWaypointToBackdropCenter;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_BackdropRelocWaypointToBackdropLeft;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_BackdropRelocWaypointToBackdropRight;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_CenterSpikeToBackdropWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_LeftSpikeToBackdropWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_RightSpikeToBackdropWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_StartToCenterSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_StartToLeftSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_StartToRightSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_CenterSpikeToStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_CenterStackToDoorWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_CenterStackToTrussWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_DoorStackWaypointToBackdropWaypointViaDoor;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_LeftSpikeToStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_RightSpikeToStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_StartToCenterSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_StartToLeftSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_StartToRightSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropLeftToBackdropWaypointTruss;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_DoorStackWaypointToBackdropWaypointViaDoorWait;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_TrussStackWaypointToBackdropWaypointViaTruss;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_TrussStackWaypointToBackdropWaypointViaTrussWait;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_TransitToBackstageViaDoor;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_TransitToBackstageViaDoorWait;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_TransitToBackstageViaTruss;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_TransitToBackstageViaTrussWait;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropCenterToBackdropLeft;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropCenterToBackdropRight;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropCenterToCenterPark;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropCenterToCornerPark;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropLeftToBackdropCenter;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropLeftToBackdropRight;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropLeftToCenterPark;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropLeftToCornerPark;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropRightToBackdropCenter;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropRightToBackdropLeft;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropRightToCenterPark;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropRightToCornerPark;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropToStackViaDoor;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropToStackViaTruss;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackstageToStackViaDoor;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackstageToStackViaTruss;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_DoorBackdropTransitWaypointToBackdropCenter;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_DoorBackdropTransitWaypointToBackdropLeft;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_DoorBackdropTransitWaypointToBackdropRight;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_TrussBackdropTransitWaypointToBackdropCenter;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_TrussBackdropTransitWaypointToBackdropLeft;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_TrussBackdropTransitWaypointToBackdropRight;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.redBackstage_StartPos;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.redWings_StartPos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Arm;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Intake;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.LEDs;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Lift;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Webcams;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Winch;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.ARTEMIS.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

//@Disabled
@Autonomous(group = "drive", name = "\uD83D\uDFE5 RedAuto2") // , preselectTeleOp="Gen1_TeleOp"
public class RedAuto2 extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private SampleMecanumDrive drive;
    private static AprilTagLibrary tags = AprilTagGameDatabase.getCurrentGameTagLibrary();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private LEDs leds;
    private Gripper gripper;
    private Wrist wrist;
    private Arm arm;
    private Lift lift;
    private Intake intake;
    private DroneLauncher launcher;
    private Winch winch;
    private Drivetrain drivetrain;
    private Webcams webcam;

    private Gamepad currentGamepad, previousGamepad;

    private String startingSide = "wing", cycleTarget = "backdrop", transitVia = "door", parkIn = "center";
    private boolean deliverYellow = true, cycle = false, wait = false;

    private static TrajectorySequence
            StartToSpike, // both
            SpikeToStack, // wing
            SpikeToBackdropYellow, // backstage
            StackToStackWaypoint, // wing, cycle
            StackWaypointToBackWaypoint, // wing, cycle
            BackWaypointToBackdropYellow, // wing, cycle
            BackWaypointToBackdropWhite, // wing, cycle
            BackdropRelocWaypointToBackdrop, // backstage
            BackWaypointToBackstage, // wing, cycle, park
            BackdropYellowSlotToWhiteSlot, // wing, cycle
            BackdropToPark, // both
            BackdropToBackdropWaypoint, // cycle
            BackdropWaypointToStackWaypoint, // cycle
            StackWaypointToStack; // cycle

    private static AprilTagMetadata targetBackdropTag;

    private boolean commandsScheduled = false;

    AutoGenerator2 autoGenerator;
    private SequentialCommandGroup autoCommands;

    @Override
    public void initialize() {
        currentGamepad = new Gamepad();
        previousGamepad = new Gamepad();

        MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        autoCommands = new SequentialCommandGroup();
        autoGenerator = new AutoGenerator2();

        drive = new SampleMecanumDrive(hardwareMap);
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
        launcher = new DroneLauncher(hardwareMap);

        gripper.grabRight();
        gripper.grabLeft();
        intake.up();
        wrist.rollToCentered();
//        wrist.tiltToIntake();
        launcher.latch();

        drivetrain.setStartingOffsetDegs(270);
//        drivetrain.setStartingError();

        webcam.setCamera("front");
        webcam.setActiveProcessor("redProp");

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
//            telemetry.addData("avgLeftBox", "%.3f", webcam.getAvgLeftBoxRed());
//            telemetry.addData("avgRightBox", "%.3f", webcam.getAvgRightBoxRed());

            telemetry.addData("raw imu degs: ", drivetrain.getRawYawDegrees());
            telemetry.addData("converted imu degs: ", drivetrain.getYawDegrees());

            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // determine target backdrop tag
            if (webcam.getPropPosition() == "left")
                targetBackdropTag = tags.lookupTag(4);
            else if (webcam.getPropPosition() == "right")
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
                if (webcam.getPropPosition() == "left") {
                    StartToSpike = RedWings_StartToLeftSpike;
                    SpikeToStack = RedWings_LeftSpikeToStack;

                    if (transitVia == "door") {
                        BackWaypointToBackdropYellow = Red_DoorBackdropTransitWaypointToBackdropLeft;
                        BackdropYellowSlotToWhiteSlot = Red_BackdropLeftToBackdropCenter;
                    } else {
                        BackWaypointToBackdropYellow = Red_TrussBackdropTransitWaypointToBackdropLeft;
                        BackdropYellowSlotToWhiteSlot = Red_BackdropLeftToBackdropRight;
                    }

                    telemetry.addLine("left spike traj");
                } else if (webcam.getPropPosition() == "right") {
                    StartToSpike = RedWings_StartToRightSpike;
                    SpikeToStack = RedWings_RightSpikeToStack;

                    if (transitVia == "door") {
                        BackWaypointToBackdropYellow = Red_DoorBackdropTransitWaypointToBackdropRight;
                        BackdropYellowSlotToWhiteSlot = Red_BackdropRightToBackdropLeft;
                    } else {
                        BackWaypointToBackdropYellow = Red_TrussBackdropTransitWaypointToBackdropRight;
                        BackdropYellowSlotToWhiteSlot = Red_BackdropRightToBackdropCenter;
                    }

                    telemetry.addLine("right spike traj");
                } else {
                    StartToSpike = RedWings_StartToCenterSpike;
                    SpikeToStack = RedWings_CenterSpikeToStack;

                    if (transitVia == "door") {
                        BackWaypointToBackdropYellow = Red_DoorBackdropTransitWaypointToBackdropCenter;
                        BackdropYellowSlotToWhiteSlot = Red_BackdropCenterToBackdropLeft;
                    } else {
                        BackWaypointToBackdropYellow = Red_TrussBackdropTransitWaypointToBackdropCenter;
                        BackdropYellowSlotToWhiteSlot = Red_BackdropCenterToBackdropRight;
                    }

                    telemetry.addLine("center spike traj");
                }

                if (transitVia == "door") {
                    StackToStackWaypoint = RedWings_CenterStackToDoorWaypoint;
                } else {
                    StackToStackWaypoint = RedWings_CenterStackToTrussWaypoint;
                }
            } else { // starting in backstage
                drive.setPoseEstimate(redBackstage_StartPos);
                if (webcam.getPropPosition() == "left") {
                    StartToSpike = RedBackstage_StartToLeftSpike;
                    SpikeToBackdropYellow = RedBackstage_LeftSpikeToBackdropWaypoint;
                    BackdropRelocWaypointToBackdrop = RedBackstage_BackdropRelocWaypointToBackdropLeft;

                    if(cycle){
                        if(transitVia == "truss") {
                            BackdropToBackdropWaypoint = Red_BackdropLeftToBackdropWaypointTruss;
//                            BackdropWaypointToStackWaypoint =
                        }
                    } else {

                    }


                    if(parkIn == "center")
                        BackdropToPark = Red_BackdropLeftToCenterPark;
                    else
                        BackdropToPark = Red_BackdropLeftToCornerPark;

                    telemetry.addLine("left spike traj");
                } else if (webcam.getPropPosition() == "right") {
                    StartToSpike = RedBackstage_StartToRightSpike;
//                    SpikeToBackdropYellow = RedBackstage_RightSpikeToBackdropRight;
                    SpikeToBackdropYellow = RedBackstage_RightSpikeToBackdropWaypoint;
                    BackdropRelocWaypointToBackdrop = RedBackstage_BackdropRelocWaypointToBackdropRight;

                    if(parkIn == "center")
                        BackdropToPark = Red_BackdropRightToCenterPark;
                    else
                        BackdropToPark = Red_BackdropRightToCornerPark;

                    telemetry.addLine("right spike traj");
                } else {
                    StartToSpike = RedBackstage_StartToCenterSpike;
//                    SpikeToBackdropYellow = RedBackstage_CenterSpikeToBackdropCenter;
                    SpikeToBackdropYellow = RedBackstage_CenterSpikeToBackdropWaypoint;
                    BackdropRelocWaypointToBackdrop = RedBackstage_BackdropRelocWaypointToBackdropCenter;

                    if(parkIn == "center")
                        BackdropToPark = Red_BackdropCenterToCenterPark;
                    else
                        BackdropToPark = Red_BackdropCenterToCornerPark;

                    telemetry.addLine("center spike traj");
                }
            }

            // determine backdrop delivery position
            if (transitVia == "door") {
                if (wait)
                    if (deliverYellow)
                        StackWaypointToBackWaypoint = Red_DoorStackWaypointToBackdropWaypointViaDoorWait;
                    else
                        StackWaypointToBackWaypoint = RedWings_TransitToBackstageViaDoorWait;
                else {
                    if (deliverYellow)
                        StackWaypointToBackWaypoint = RedWings_DoorStackWaypointToBackdropWaypointViaDoor;
                    else
                        StackWaypointToBackWaypoint = RedWings_TransitToBackstageViaDoor;
                }

                if (cycleTarget == "backdrop")
                    BackdropWaypointToStackWaypoint = Red_BackdropToStackViaDoor;
                else
                    BackdropWaypointToStackWaypoint = Red_BackstageToStackViaDoor;
            } else { // transiting via truss
                if (wait)
                    if (deliverYellow)
                        StackWaypointToBackWaypoint = RedWings_TrussStackWaypointToBackdropWaypointViaTrussWait;
                    else
                        StackWaypointToBackWaypoint = RedWings_TransitToBackstageViaTrussWait;
                else {
                    if (deliverYellow)
                        StackWaypointToBackWaypoint = RedWings_TrussStackWaypointToBackdropWaypointViaTruss;
                    else
                        StackWaypointToBackWaypoint = RedWings_TransitToBackstageViaTruss;
                }

                if (cycleTarget == "backdrop")
                    BackdropWaypointToStackWaypoint = Red_BackdropToStackViaTruss;
                else
                    BackdropWaypointToStackWaypoint = Red_BackstageToStackViaTruss;
            }

            telemetry.addLine("waitForStart");
            telemetry.addData("Prop Position", webcam.getPropPosition());
            telemetry.update();
            sleep(20);
        }

        if (isStopRequested()) {
//            portal.close();
            return;
        }

        if (isStarted() && !commandsScheduled) {
            // drop purple
            /*
            GenerateAutoCommands passes these parameters

            Arm arm, Wrist wrist, Gripper gripper, Lift lift, Intake intake, Winch winch, LEDs leds,
            Drivetrain drivetrain, Webcams webcam, AprilTagMetadata targetBackdropTag,
            SampleMecanumDrive drive,

            TrajectorySequence StartToSpike, // both
            TrajectorySequence SpikeToStack, // wing
            TrajectorySequence SpikeToBackdropYellow, // backstage
            TrajectorySequence StackToStackWaypoint, // wing, cycle
            TrajectorySequence StackWaypointToBackWaypoint, // wing, cycle
            TrajectorySequence BackWaypointToBackdropYellow, // wing, cycle
            TrajectorySequence BackWaypointToBackdropWhite, // wing, cycle
    TrajectorySequence BackdropRelocWaypointToBackdrop, //backstage
            TrajectorySequence BackWaypointToBackstage, // wing, cycle, park
            TrajectorySequence BackdropYellowSlotToWhiteSlot, // wing, cycle
            TrajectorySequence BackdropToPark, // both
            TrajectorySequence BackdropToBackdropWaypoint, // cycle
            TrajectorySequence BackdropWaypointToStackWaypoint, // cycle
            TrajectorySequence StackWaypointToStack, // cycle

            String startingSide, String cycleTarget, String transitVia, String parkIn, boolean cycle, boolean wait

            */

            schedule(
                    autoGenerator.generateAutoCommands2(
                            arm, wrist, gripper, lift, intake, winch, leds, drivetrain, webcam,
                            targetBackdropTag, webcam.getActiveAprilTagProcessor(),
                            drive,
                            StartToSpike, // both
                            SpikeToStack, // wing
                            SpikeToBackdropYellow, // backstage
                            StackToStackWaypoint, // wing, cycle
                            StackWaypointToBackWaypoint, // wing, cycle
                            BackWaypointToBackdropYellow, // wing, cycle
                            BackWaypointToBackdropWhite, // wing, cycle
                            BackdropRelocWaypointToBackdrop, //backstage
                            BackWaypointToBackstage, // wing, cycle, park
                            BackdropYellowSlotToWhiteSlot, // wing, cycle
                            BackdropToPark, // both
                            BackdropToBackdropWaypoint, // cycle
                            BackdropWaypointToStackWaypoint, // cycle
                            StackWaypointToStack, // cycle
                            "red", startingSide, cycleTarget, transitVia, parkIn, cycle,
                            wait, deliverYellow, telemetry
                    )
            );
            webcam.setCamera("back");
            webcam.setActiveProcessor("apriltag");
            commandsScheduled = true;
        }

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            telemetry.addData("liftBase", lift.getLiftBase());
            telemetry.addLine();
            telemetry.addData("heading degs", drivetrain.getYawDegrees());
            telemetry.addData("converted heading ", (drivetrain.getYawDegrees() + 360) % 360);
            telemetry.addLine();
            telemetry.addData("heading rads", drivetrain.getYawRadians());
            telemetry.addLine();
            telemetry.addData("poseEstimate", drive.getPoseEstimate());
            telemetry.update();
        }

        telemetry.update();
    }
}