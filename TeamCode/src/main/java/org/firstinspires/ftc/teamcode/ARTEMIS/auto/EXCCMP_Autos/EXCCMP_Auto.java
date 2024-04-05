package org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos;


import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.DoorBackdropWaypointToStackWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.DoorStackWaypointToCenterStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.DoorStackWaypointToInnerStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.DoorStackWaypointToOuterStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_BackdropRelocWaypointToBackdropCenter;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_BackdropRelocWaypointToBackdropLeft;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_BackdropRelocWaypointToBackdropRight;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_CenterSpikeToBackdropWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_LeftSpikeToBackdropWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_RightSpikeToBackdropWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_StartToCenterSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_StartToLeftSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_StartToRightSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_CenterSpikeToCenterStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_CenterSpikeToInnerStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_CenterSpikeToOuterStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_CenterStackToDoorWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_CenterStackToTrussWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_DoorStackWaypointToBackdropWaypointViaDoor;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_InnerStackToDoorWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_InnerStackToTrussWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_LeftSpikeToCenterStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_LeftSpikeToInnerStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_LeftSpikeToOuterStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_OuterStackToDoorWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_OuterStackToTrussWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_RightSpikeToCenterStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_RightSpikeToInnerStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_RightSpikeToOuterStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_StartToCenterSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_StartToLeftSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_StartToRightSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropCenterToBackdropWaypointDoor;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropCenterToBackdropWaypointTruss;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropLeftToBackdropWaypointDoor;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropLeftToBackdropWaypointTruss;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropRightToBackdropWaypointDoor;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_BackdropRightToBackdropWaypointTruss;
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
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_DoorBackdropTransitWaypointToBackdropCenter;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_DoorBackdropTransitWaypointToBackdropLeft;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_DoorBackdropTransitWaypointToBackdropRight;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_TrussBackdropTransitWaypointToBackdropCenter;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_TrussBackdropTransitWaypointToBackdropLeft;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.Red_TrussBackdropTransitWaypointToBackdropRight;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.TrussBackdropWaypointToStackWaypoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.TrussStackWaypointToCenterStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.TrussStackWaypointToInnerStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.TrussStackWaypointToOuterStack;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.blueBackstage_StartPos;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.blueWings_StartPos;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.redBackstage_StartPos;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.redWings_StartPos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
@Autonomous(group = "auto", name = "Champs Auto", preselectTeleOp="Gen1_TeleOp") // , preselectTeleOp="Gen1_TeleOp" //🟥
public class EXCCMP_Auto extends CommandOpMode {
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

    public static String startingSide = "wing", cycleTarget = "backdrop", transitVia = "door", cycleVia = "door", targetStack = "inner", parkIn = "center", alliance = "red", yellowTo = "center";
    public static boolean park = false, deliverYellow = true, cycle = true, wait = false;

    //2*2*2*2*3*3*2*3*2*2*2*2

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

    AutoGenerator autoGenerator;
    private SequentialCommandGroup autoCommands;

    private ElapsedTime imuRelocTimer;

    @Override
    public void initialize() {
        currentGamepad = new Gamepad();
        previousGamepad = new Gamepad();

        MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        imuRelocTimer = new ElapsedTime();

        autoCommands = new SequentialCommandGroup();
        autoGenerator = new AutoGenerator();

        SampleMecanumDrive.flipPose = true;

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

//        drivetrain.setStartingOffsetDegs(90);
//        drivetrain.setStartingError();

        webcam.setCamera("front");
        webcam.setActiveProcessor("blueProp");

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
//            telemetry.addData("avgLeftBox", "%.3f", webcam.getAvgLeftBoxBlue());
//            telemetry.addData("avgRightBox", "%.3f", webcam.getAvgRightBoxBlue());
//
//            telemetry.addData("raw imu degs: ", drivetrain.getRawYawDegrees());
//            telemetry.addData("converted imu degs: ", drivetrain.getYawDegrees());

            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // determine target backdrop tag
            if (alliance == "red") {
                if (webcam.getPropPosition() == "left")
                    targetBackdropTag = tags.lookupTag(4);
                else if (webcam.getPropPosition() == "right")
                    targetBackdropTag = tags.lookupTag(6);
                else
                    targetBackdropTag = tags.lookupTag(5);
            } else {
                if (webcam.getPropPosition() == "left")
                    targetBackdropTag = tags.lookupTag(1);
                else if (webcam.getPropPosition() == "right")
                    targetBackdropTag = tags.lookupTag(3);
                else
                    targetBackdropTag = tags.lookupTag(2);
            }

            // red blue 🔵🟥

            /*

    Driver
        Trigger Left -
        Trigger Right -

        Bumper Left - alliance
        Bumper Right - startingSide

        DPAD Up - targetStack
        DPAD Right - cycleTarget
        DPAD Down - transitVia
        DPAD Left - cycleVia

        Y/Tri - park
        A/Cross - cycle
        B/Circle - wait
        X/square - parkIn

        Back/Share - deliverYellow

    public static String startingSide = "wing", cycleTarget = "backdrop", transitVia = "door", cycleVia = "door", targetStack = "center", parkIn = "center", alliance = "red";
    public static boolean park = true, deliverYellow = true, cycle = false, wait = false;

    Manipulator

 */

//            if(!SampleMecanumDrive.flipPose)
//                telemetry.addLine("RIGHT | alliance: RED");
//            else
//                telemetry.addLine("RIGHT | alliance: BLUE");
            telemetry.addData("B-LEFT | alliance: ", alliance);
            if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
                SampleMecanumDrive.flipPose = !SampleMecanumDrive.flipPose;
                EXCCMP_AutoTrajectories.generateTrajectories(drive);
                if (!SampleMecanumDrive.flipPose) // red
                    webcam.setActiveProcessor("redProp");
                else
                    webcam.setActiveProcessor("blueProp");
            }

            if (!SampleMecanumDrive.flipPose) {
                alliance = "red";
            } else {
                alliance = "blue";
            }

            telemetry.addData("⬅️ LEFT | startingSide: ", startingSide);
            if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
                if (startingSide != "wing")
                    startingSide = "wing";
                else
                    startingSide = "backstage";
                EXCCMP_AutoTrajectories.generateTrajectories(drive);
            }

            telemetry.addData("UP | cycle y/n: ", cycle);
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up)
                cycle = !cycle;

            telemetry.addData("A | park y/n: ", park);
            if (currentGamepad.a && !previousGamepad.a)
                park = !park;

            if (park) {
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

            telemetry.addData("BACK | deliverYellow: ", deliverYellow);
            if (currentGamepad.back && !previousGamepad.back)
                deliverYellow = !deliverYellow;

            telemetry.addData("Y | TRANSIT: ", transitVia);
            if (currentGamepad.y && !previousGamepad.y) {
                if (transitVia != "door")
                    transitVia = "door";
                else
                    transitVia = "truss";
            }

            telemetry.addData("LEFT STICK BUTTON | CycleVia: ", cycleVia);
            if (currentGamepad.left_stick_button && !previousGamepad.left_stick_button) {
                if (cycleVia != "door")
                    cycleVia = "door";
                else
                    cycleVia = "truss";
            }

            telemetry.addData("⬇️ DOWN | CYCLE TARGET: ", cycleTarget);
            if (currentGamepad.dpad_down && !previousGamepad.dpad_down)
                if (cycleTarget != "backstage")
                    cycleTarget = "backstage";
                else
                    cycleTarget = "backdrop";

            telemetry.addData("RIGHTSTICK | TARGET STACK: ", targetStack);
            if (currentGamepad.right_stick_button && !previousGamepad.right_stick_button)
                if (targetStack == "inner")
                    targetStack = "outer";
                else if (targetStack == "outer")
                    targetStack = "center";
                else
                    targetStack = "inner";

            telemetry.addData("START | yellowTo: ", yellowTo);
            if (currentGamepad.start && !previousGamepad.start)
                if (yellowTo == "inner")
                    yellowTo = "outer";
                else if (yellowTo == "outer")
                    yellowTo = "center";
                else
                    yellowTo = "inner";

            if (alliance == "red")
                drivetrain.setStartingOffsetDegs(270);
            else
                drivetrain.setStartingOffsetDegs(90);

            //determine trajectories
            if (startingSide.toLowerCase() == "wing") {
                if (alliance == "red")
                    drive.setPoseEstimate(redWings_StartPos);
                else
                    drive.setPoseEstimate(blueWings_StartPos);

                //determine wing trajectories
                if (webcam.getPropPosition() == "left") {
                    if (alliance == "red") {
                        StartToSpike = RedWings_StartToLeftSpike;

                        if (targetStack == "inner")
                            SpikeToStack = RedWings_LeftSpikeToInnerStack;
                        else if (targetStack == "outer")
                            SpikeToStack = RedWings_LeftSpikeToOuterStack;
                        else
                            SpikeToStack = RedWings_LeftSpikeToCenterStack;

                    } else {
                        StartToSpike = RedWings_StartToRightSpike;
                        //HERE

                        if (targetStack == "inner")
                            SpikeToStack = RedWings_RightSpikeToInnerStack;
                        else if (targetStack == "outer")
                            SpikeToStack = RedWings_RightSpikeToOuterStack;
                        else
                            SpikeToStack = RedWings_RightSpikeToCenterStack;

                    }

                    if (transitVia == "door") {
                        if (alliance == "red") {
                            BackWaypointToBackdropYellow = Red_DoorBackdropTransitWaypointToBackdropLeft;
                            BackdropYellowSlotToWhiteSlot = Red_BackdropLeftToBackdropCenter;
                        } else {
                            BackWaypointToBackdropYellow = Red_DoorBackdropTransitWaypointToBackdropRight;
                            BackdropYellowSlotToWhiteSlot = Red_BackdropRightToBackdropCenter;
                        }
                    } else {
                        if (alliance == "red") {
                            BackWaypointToBackdropYellow = Red_TrussBackdropTransitWaypointToBackdropLeft;
                            BackdropYellowSlotToWhiteSlot = Red_BackdropLeftToBackdropRight;
                        } else {
                            BackWaypointToBackdropYellow = Red_TrussBackdropTransitWaypointToBackdropRight;
                            BackdropYellowSlotToWhiteSlot = Red_BackdropRightToBackdropLeft;
                        }
                    }

                    telemetry.addLine("left spike traj");
                } else if (webcam.getPropPosition() == "right") {
                    if (alliance == "red") {
                        StartToSpike = RedWings_StartToRightSpike;

                        if (targetStack == "inner")
                            SpikeToStack = RedWings_RightSpikeToInnerStack;
                        else if (targetStack == "outer")
                            SpikeToStack = RedWings_RightSpikeToOuterStack;
                        else
                            SpikeToStack = RedWings_RightSpikeToCenterStack;
                    } else {
                        StartToSpike = RedWings_StartToLeftSpike;

                        if (targetStack == "inner")
                            SpikeToStack = RedWings_LeftSpikeToInnerStack;
                        else if (targetStack == "outer")
                            SpikeToStack = RedWings_LeftSpikeToOuterStack;
                        else
                            SpikeToStack = RedWings_LeftSpikeToCenterStack;
//                        SpikeToStack = RedWings_LeftSpikeToCenterStack;
                    }

                    if (transitVia == "door") {
                        if (alliance == "red") {
                            BackWaypointToBackdropYellow = Red_DoorBackdropTransitWaypointToBackdropRight;
                            BackdropYellowSlotToWhiteSlot = Red_BackdropRightToBackdropLeft;
                        } else {
                            BackWaypointToBackdropYellow = Red_DoorBackdropTransitWaypointToBackdropLeft;
                            BackdropYellowSlotToWhiteSlot = Red_BackdropLeftToBackdropRight;
                        }
                    } else {
                        if (alliance == "red") {
                            BackWaypointToBackdropYellow = Red_TrussBackdropTransitWaypointToBackdropRight;
                            BackdropYellowSlotToWhiteSlot = Red_BackdropRightToBackdropCenter;
                        } else {
                            BackWaypointToBackdropYellow = Red_TrussBackdropTransitWaypointToBackdropLeft;
                            BackdropYellowSlotToWhiteSlot = Red_BackdropLeftToBackdropCenter;
                        }
                    }

                    telemetry.addLine("right spike traj");
                } else {
                    StartToSpike = RedWings_StartToCenterSpike;
                    if (targetStack == "inner")
                        SpikeToStack = RedWings_CenterSpikeToInnerStack;
                    else if (targetStack == "outer")
                        SpikeToStack = RedWings_CenterSpikeToOuterStack;
                    else
                        SpikeToStack = RedWings_CenterSpikeToCenterStack;

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
//                    StackToStackWaypoint = RedWings_CenterStackToDoorWaypoint;
                    if (targetStack == "inner") {
                        StackToStackWaypoint = RedWings_InnerStackToDoorWaypoint;
                    } else if (targetStack == "outer") {
                        StackToStackWaypoint = RedWings_OuterStackToDoorWaypoint;
                    } else {
                        StackToStackWaypoint = RedWings_CenterStackToDoorWaypoint;
                    }
                } else {
//                    StackToStackWaypoint = RedWings_CenterStackToTrussWaypoint;

                    if (targetStack == "inner") {
                        StackToStackWaypoint = RedWings_InnerStackToTrussWaypoint;
                    } else if (targetStack == "outer") {
                        StackToStackWaypoint = RedWings_OuterStackToTrussWaypoint;
                    } else {
                        StackToStackWaypoint = RedWings_CenterStackToTrussWaypoint;
                    }
                }
            } else { // starting in backstage
                if (alliance == "red")
                    drive.setPoseEstimate(redBackstage_StartPos);
                else
                    drive.setPoseEstimate(blueBackstage_StartPos);

                if (webcam.getPropPosition() == "left") {
                    if (alliance == "red") {
                        StartToSpike = RedBackstage_StartToLeftSpike;
                        SpikeToBackdropYellow = RedBackstage_LeftSpikeToBackdropWaypoint;
                        BackdropRelocWaypointToBackdrop = RedBackstage_BackdropRelocWaypointToBackdropLeft;
                    } else {
                        StartToSpike = RedBackstage_StartToRightSpike;
                        SpikeToBackdropYellow = RedBackstage_RightSpikeToBackdropWaypoint;
                        BackdropRelocWaypointToBackdrop = RedBackstage_BackdropRelocWaypointToBackdropRight;
                    }

                    telemetry.addLine("left spike traj");
                } else if (webcam.getPropPosition() == "right") {
//                    SpikeToBackdropYellow = RedBackstage_RightSpikeToBackdropRight;
                    if (alliance == "red") {
                        StartToSpike = RedBackstage_StartToRightSpike;
                        SpikeToBackdropYellow = RedBackstage_RightSpikeToBackdropWaypoint;
                        BackdropRelocWaypointToBackdrop = RedBackstage_BackdropRelocWaypointToBackdropRight;
                    } else {
                        StartToSpike = RedBackstage_StartToLeftSpike;
                        SpikeToBackdropYellow = RedBackstage_LeftSpikeToBackdropWaypoint;
                        BackdropRelocWaypointToBackdrop = RedBackstage_BackdropRelocWaypointToBackdropLeft;
                    }


                    telemetry.addLine("right spike traj");
                } else {
//                    SpikeToBackdropYellow = RedBackstage_CenterSpikeToBackdropCenter;
                    StartToSpike = RedBackstage_StartToCenterSpike;
                    SpikeToBackdropYellow = RedBackstage_CenterSpikeToBackdropWaypoint;
                    BackdropRelocWaypointToBackdrop = RedBackstage_BackdropRelocWaypointToBackdropCenter;

                    telemetry.addLine("center spike traj");
                }
            }

            if (cycle) {
                if (cycleTarget == "backdrop") {
                    if (webcam.getPropPosition() == "right") {
//                        if (transitVia == "door")
//                            BackWaypointToBackdropWhite = Red_DoorBackdropTransitWaypointToBackdropLeft;
//                        else
                        BackWaypointToBackdropWhite = Red_DoorBackdropTransitWaypointToBackdropCenter;
                    } else if (webcam.getPropPosition() == "center") {
                        if (transitVia == "door")
                            BackWaypointToBackdropWhite = Red_DoorBackdropTransitWaypointToBackdropLeft;
                        else
                            BackWaypointToBackdropWhite = Red_DoorBackdropTransitWaypointToBackdropRight;
                    } else {
//                        if (transitVia == "door")
                        BackWaypointToBackdropWhite = Red_DoorBackdropTransitWaypointToBackdropCenter;
//                        else
//                            BackWaypointToBackdropWhite = Red_DoorBackdropTransitWaypointToBackdropRight;
                    }
                } else { //cycle target backstage
                    if (transitVia == "door") {
                        BackWaypointToBackstage = RedWings_TransitToBackstageViaDoor;
                    } else {
                        BackWaypointToBackstage = RedWings_TransitToBackstageViaTruss;
                    }
                }

                if (webcam.getPropPosition() == "left") {
                    if (transitVia == "truss") {
                        BackdropToBackdropWaypoint = Red_BackdropLeftToBackdropWaypointTruss;
                    } else
                        BackdropToBackdropWaypoint = Red_BackdropLeftToBackdropWaypointDoor;
                } else if (webcam.getPropPosition() == "right") {
                    if (transitVia == "truss") {
                        BackdropToBackdropWaypoint = Red_BackdropRightToBackdropWaypointTruss;
                    } else
                        BackdropToBackdropWaypoint = Red_BackdropRightToBackdropWaypointDoor;
                } else {
                    if (transitVia == "truss") {
                        BackdropToBackdropWaypoint = Red_BackdropCenterToBackdropWaypointTruss;
                    } else
                        BackdropToBackdropWaypoint = Red_BackdropCenterToBackdropWaypointDoor;
                }

                if (cycleVia == "door") {
                    BackdropWaypointToStackWaypoint = DoorBackdropWaypointToStackWaypoint;

                    if (targetStack == "inner") {
                        StackWaypointToStack = DoorStackWaypointToInnerStack;
                        StackToStackWaypoint = RedWings_InnerStackToDoorWaypoint;
                    } else if (targetStack == "outer") {
                        StackWaypointToStack = DoorStackWaypointToOuterStack;
                        StackToStackWaypoint = RedWings_OuterStackToDoorWaypoint;
                    } else {
                        StackWaypointToStack = DoorStackWaypointToCenterStack;
                        StackToStackWaypoint = RedWings_CenterStackToDoorWaypoint;
                    }
                } else {
                    BackdropWaypointToStackWaypoint = TrussBackdropWaypointToStackWaypoint;

                    if (targetStack == "inner") {
                        StackWaypointToStack = TrussStackWaypointToInnerStack;
                        StackToStackWaypoint = RedWings_InnerStackToTrussWaypoint;
                    } else if (targetStack == "outer") {
                        StackWaypointToStack = TrussStackWaypointToOuterStack;
                        StackToStackWaypoint = RedWings_OuterStackToTrussWaypoint;
                    } else {
                        StackWaypointToStack = TrussStackWaypointToCenterStack;
                        StackToStackWaypoint = RedWings_CenterStackToTrussWaypoint;
                    }
                }
            }

            if (park) {
                if (webcam.getPropPosition() == "left") {
                    if (parkIn == "center") {
                        if (alliance == "red")
                            BackdropToPark = Red_BackdropLeftToCenterPark;
                        else
                            BackdropToPark = Red_BackdropRightToCenterPark;
                    } else {
                        if (alliance == "red")
                            BackdropToPark = Red_BackdropLeftToCornerPark;
                        else
                            BackdropToPark = Red_BackdropRightToCornerPark;
                    }
                } else if (webcam.getPropPosition() == "right") {
                    if (parkIn == "center") {
                        if (alliance == "red")
                            BackdropToPark = Red_BackdropRightToCenterPark;
                        else
                            BackdropToPark = Red_BackdropLeftToCenterPark;
                    } else {
                        if (alliance == "red")
                            BackdropToPark = Red_BackdropRightToCornerPark;
                        else
                            BackdropToPark = Red_BackdropLeftToCornerPark;
                    }
                } else {
                    if (parkIn == "center")
                        BackdropToPark = Red_BackdropCenterToCenterPark;
                    else
                        BackdropToPark = Red_BackdropCenterToCornerPark;
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

//                if (cycleTarget == "backdrop")
//                    BackdropWaypointToStackWaypoint = Red_BackdropToStackViaDoor;
//                else
//                    BackdropWaypointToStackWaypoint = Red_BackstageToStackViaDoor;
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

//                if (cycleTarget == "backdrop")
//                    BackdropWaypointToStackWaypoint = Red_BackdropToStackViaTruss;
//                else
//                    BackdropWaypointToStackWaypoint = Red_BackstageToStackViaTruss;
            }

            telemetry.addData("converted imu degs: ", drivetrain.getYawDegrees());
            telemetry.addData("offset in degs: ", drivetrain.getStartingOffsetDegs());
            telemetry.addData("raw imu degs: ", drivetrain.getRawYawDegrees());
            telemetry.addLine();
            telemetry.addLine("waitForStart");
            telemetry.addLine();
            telemetry.addData("flipPose", SampleMecanumDrive.flipPose);
            telemetry.addData("Prop Position", webcam.getPropPosition());
            telemetry.addLine("target backdrop tag: " + targetBackdropTag.name);
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
            telemetry.addData("converted heading ", drivetrain.getPose2dYawDegs());
            telemetry.addLine();
            telemetry.addData("heading rads", drivetrain.getYawRadians());
            telemetry.addData("heading rads", drivetrain.getPose2dYawRads());
            telemetry.addLine();
            telemetry.addData("poseEstimate", drive.getPoseEstimate());
            telemetry.update();

            if(imuRelocTimer.seconds() >= 20){
//                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians((drivetrain.getYawDegrees() + 360) % 360)));
                telemetry.addData("relocalized using imu ", (drivetrain.getYawDegrees() + 360) % 360);
                imuRelocTimer.reset();
            }

        }

        telemetry.update();
    }

    public static String getStartingSide() {
        return startingSide;
    }
}