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
import org.firstinspires.ftc.teamcode.TESTBED.visionTesting.RedPropDetection;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

//public class CSTB_redWings_Park {
@Disabled
@Autonomous(group = "drive", name = "CSTB red wings PARK")
public class CSTB_redWings_Park extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private CSTB_SampleMecanumDrive drive;
    private VisionPortal portal;
    private RedPropDetection redPropThreshold;
    //    private Lift lift;
    TrajectorySequence redWings_DecisionPointToSpike, redWings_ToDecisionPointTrajectory, redWings_SpikeToDecisionPoint;
    FtcDashboard dashboard = FtcDashboard.getInstance();

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

        drive = new CSTB_SampleMecanumDrive(hardwareMap);


        drive.setPoseEstimate(CSTB_AutoTrajectories.CSTB_redWings_StartPos);
        CSTB_AutoTrajectories.generateTrajectories(drive);

        //gripper.close();
////////////////////////////DEFINING PARK TRAJECTORIES//////////////////////////////

        ////////////////////////////////////DONE DEFINING PARK TRAJECTORIES///////////////////////////////////////


        switch (redPropThreshold.getPropPosition()) {
            case "left":
                redWings_DecisionPointToSpike = CSTB_AutoTrajectories.CSTB_redWings_DecisionPointToLeftSpike;
                redWings_SpikeToDecisionPoint = CSTB_AutoTrajectories.CSTB_redWings_LeftSpikeToDecisionPoint;
                telemetry.addLine("park traj 1");
                break;
            default:
            case "center":
                redWings_DecisionPointToSpike = CSTB_AutoTrajectories.CSTB_redWings_DecisionPointToCenterSpike;
                redWings_SpikeToDecisionPoint = CSTB_AutoTrajectories.CSTB_redWings_CenterSpikeToDecisionPoint;
                telemetry.addLine("park traj 2");
                break;
            case "right":
                redWings_DecisionPointToSpike = CSTB_AutoTrajectories.CSTB_redWings_DecisionPointToRightSpike;
                redWings_SpikeToDecisionPoint = CSTB_AutoTrajectories.CSTB_redWings_RightSpikeToDecisionPoint;
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

                new CSTB_FollowTrajectoryCommand(drive, CSTB_AutoTrajectories.CSTB_redWings_StartPositionToDecisionPoint),
                new CSTB_FollowTrajectoryCommand(drive, redWings_DecisionPointToSpike),
                new WaitCommand(1000),
                new CSTB_FollowTrajectoryCommand(drive, redWings_SpikeToDecisionPoint),
                new CSTB_FollowTrajectoryCommand(drive, CSTB_AutoTrajectories.CSTB_redWings_DecisionPointToMiddlePark)

        ));

    }
}

