package org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024;

import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.redWings_CenterSpikeToDecisionPoint;
import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.redWings_DecisionPointToCenterSpike;
import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.redWings_LeftSpikeToDecisionPoint;
import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.redWings_RightSpikeToDecisionPoint;
import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.redWings_StartPositionToDecisionPoint;
import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.redWings_DecisionPointToLeftSpike;
import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.redWings_DecisionPointToMiddlePark;
import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.redWings_DecisionPointToRightSpike;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.visionTesting.RedPropDetection;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

//public class CSTB_redWings_Park {

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


        drive.setPoseEstimate(CSTB_AutoTrajectories.redWings_StartPos);
        CSTB_AutoTrajectories.generateTrajectories(drive);

        //gripper.close();
////////////////////////////DEFINING PARK TRAJECTORIES//////////////////////////////

        ////////////////////////////////////DONE DEFINING PARK TRAJECTORIES///////////////////////////////////////


        switch (redPropThreshold.getPropPosition()) {
            case "left":
                redWings_DecisionPointToSpike = redWings_DecisionPointToLeftSpike;
                redWings_SpikeToDecisionPoint = redWings_LeftSpikeToDecisionPoint;
                telemetry.addLine("park traj 1");
                break;
            default:
            case "center":
                redWings_DecisionPointToSpike = redWings_DecisionPointToCenterSpike;
                redWings_SpikeToDecisionPoint = redWings_CenterSpikeToDecisionPoint;
                telemetry.addLine("park traj 2");
                break;
            case "right":
                redWings_DecisionPointToSpike = redWings_DecisionPointToRightSpike;
                redWings_SpikeToDecisionPoint = redWings_RightSpikeToDecisionPoint;
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

                new CSTB_FollowTrajectoryCommand(drive, redWings_StartPositionToDecisionPoint),
                new CSTB_FollowTrajectoryCommand(drive, redWings_DecisionPointToSpike),
                new WaitCommand(1000),
                new CSTB_FollowTrajectoryCommand(drive, redWings_SpikeToDecisionPoint),
                new CSTB_FollowTrajectoryCommand(drive, redWings_DecisionPointToMiddlePark)

        ));

    }
}

