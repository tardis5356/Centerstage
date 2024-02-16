package org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleParkAutos;

import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleParkAutos.Artemis_PurpleParkAutoTrajectories.blueBackstage_CenterSpikeToDecisionPoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleParkAutos.Artemis_PurpleParkAutoTrajectories.blueBackstage_DecisionPointToCenterSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleParkAutos.Artemis_PurpleParkAutoTrajectories.blueBackstage_DecisionPointToCornerPark;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleParkAutos.Artemis_PurpleParkAutoTrajectories.blueBackstage_DecisionPointToLeftSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleParkAutos.Artemis_PurpleParkAutoTrajectories.blueBackstage_DecisionPointToRightSpike;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleParkAutos.Artemis_PurpleParkAutoTrajectories.blueBackstage_LeftSpikeToDecisionPoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleParkAutos.Artemis_PurpleParkAutoTrajectories.blueBackstage_RightSpikeToDecisionPoint;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleParkAutos.Artemis_PurpleParkAutoTrajectories.blueBackstage_StartPos;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_PurpleParkAutos.Artemis_PurpleParkAutoTrajectories.blueBackstage_StartPositionToDecisionPoint;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.ARTEMIS.auto.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Intake;
import org.firstinspires.ftc.teamcode.ARTEMIS.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.ARTEMIS.visionTesting.BluePropDetection;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

//public class CSTB_redWings_Park {
//@Disabled
@Autonomous(group = "drive", name = "blueBackstage Purple+Park")
public class blueBackstage_PurpleParkAuto extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private SampleMecanumDrive drive;
    private VisionPortal portal;
    private BluePropDetection bluePropThreshold;
    //    private Lift lift;
    public static TrajectorySequence blueBackstage_DecisionPointToSpike, blueBackstage_SpikeToDecisionPoint;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize() {
//        MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ////////‼️‼️⁉️⁉️CAMERA INITIALIZATION/DEFINING ⁉️⁉️⁉️
//        public void runOpMode () throw InterruptedException {
        bluePropThreshold = new BluePropDetection();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(redPropThreshold)
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
        Intake intake = new Intake(hardwareMap);


        drive.setPoseEstimate(blueBackstage_StartPos);
        Artemis_PurpleParkAutoTrajectories.generateTrajectories(drive);

        //gripper.close();
////////////////////////////DEFINING PARK TRAJECTORIES//////////////////////////////

        ////////////////////////////////////DONE DEFINING PARK TRAJECTORIES///////////////////////////////////////


        switch (bluePropThreshold.getPropPosition()) {
            case "left":
                blueBackstage_DecisionPointToSpike = blueBackstage_DecisionPointToLeftSpike;
                blueBackstage_SpikeToDecisionPoint = blueBackstage_LeftSpikeToDecisionPoint;
                telemetry.addLine("park traj 1");
                break;
            default:
            case "center":
                blueBackstage_DecisionPointToSpike = blueBackstage_DecisionPointToCenterSpike;
                blueBackstage_SpikeToDecisionPoint = blueBackstage_CenterSpikeToDecisionPoint;
                telemetry.addLine("park traj 2");
                break;
            case "right":
                blueBackstage_DecisionPointToSpike = blueBackstage_DecisionPointToRightSpike;
                blueBackstage_SpikeToDecisionPoint = blueBackstage_RightSpikeToDecisionPoint;
                telemetry.addLine("park traj 3");
                break;
        }

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {

            telemetry.addLine("waitForStart");
            telemetry.addData("Prop Position", bluePropThreshold.getPropPosition());
            telemetry.update();
            sleep(20);
        }


        telemetry.update();
        schedule(new SequentialCommandGroup(

                new FollowTrajectoryCommand(drive, blueBackstage_StartPositionToDecisionPoint),
                new FollowTrajectoryCommand(drive, blueBackstage_DecisionPointToSpike),
                new InstantCommand(intake::out),
                new WaitCommand(700),
                new InstantCommand(intake::stop),
                new FollowTrajectoryCommand(drive, blueBackstage_SpikeToDecisionPoint),
                new FollowTrajectoryCommand(drive, blueBackstage_DecisionPointToCornerPark)

        ));

    }
}