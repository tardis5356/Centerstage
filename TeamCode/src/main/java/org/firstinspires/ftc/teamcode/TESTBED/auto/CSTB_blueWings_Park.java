package org.firstinspires.ftc.teamcode.TESTBED.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.TESTBED.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TESTBED.visionTesting.BluePropDetection;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

//public class CSTB_redWings_Park {
@Disabled
@Autonomous(group = "drive", name = "CSTB blue wings PARK")
public class CSTB_blueWings_Park extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private CSTB_SampleMecanumDrive drive;
    private VisionPortal portal;
    private BluePropDetection bluePropThreshold;
    //    private Lift lift;
    TrajectorySequence blueWings_DecisionPointToSpike, blueWings_ToDecisionPointTrajectory, blueWings_SpikeToDecisionPoint;
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

        drive = new CSTB_SampleMecanumDrive(hardwareMap);


        drive.setPoseEstimate(CSTB_AutoTrajectories.CSTB_blueWings_StartPos);
        CSTB_AutoTrajectories.generateTrajectories(drive);

        //gripper.close();
////////////////////////////DEFINING PARK TRAJECTORIES//////////////////////////////

        ////////////////////////////////////DONE DEFINING PARK TRAJECTORIES///////////////////////////////////////


        switch (bluePropThreshold.getPropPosition()) {
            case "left":
                blueWings_DecisionPointToSpike = CSTB_AutoTrajectories.CSTB_blueWings_DecisionPointToLeftSpike;
                blueWings_SpikeToDecisionPoint = CSTB_AutoTrajectories.CSTB_blueWings_LeftSpikeToDecisionPoint;
                telemetry.addLine("park traj 1");
                break;
            default:
            case "center":
                blueWings_DecisionPointToSpike = CSTB_AutoTrajectories.CSTB_blueWings_DecisionPointToCenterSpike;
                blueWings_SpikeToDecisionPoint = CSTB_AutoTrajectories.CSTB_blueWings_CenterSpikeToDecisionPoint;
                telemetry.addLine("park traj 2");
                break;
            case "right":
                blueWings_DecisionPointToSpike = CSTB_AutoTrajectories.CSTB_blueWings_DecisionPointToRightSpike;
                blueWings_SpikeToDecisionPoint = CSTB_AutoTrajectories.CSTB_blueWings_RightSpikeToDecisionPoint;
                telemetry.addLine("park traj 3");
                break;
        }

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {

            telemetry.addLine("waitForStart");
            telemetry.update();
            sleep(20);
        }


        telemetry.update();
        schedule(new SequentialCommandGroup(

                new CSTB_FollowTrajectoryCommand(drive, CSTB_AutoTrajectories.CSTB_blueWings_StartPositionToDecisionPoint),
                new CSTB_FollowTrajectoryCommand(drive, blueWings_DecisionPointToSpike),
                new WaitCommand(1000),
                new CSTB_FollowTrajectoryCommand(drive, blueWings_SpikeToDecisionPoint),
                new CSTB_FollowTrajectoryCommand(drive, CSTB_AutoTrajectories.CSTB_blueWings_DecisionPointToMiddlePark)

        ));

    }
}

