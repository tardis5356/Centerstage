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
@Autonomous(group = "drive", name = "CSTB blue backstage PARK")
public class CSTB_redBackstage_Park extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private CSTB_SampleMecanumDrive drive;
    private VisionPortal portal;
    private RedPropDetection redPropThreshold;
    //    private Lift lift;
    TrajectorySequence redBackstage_DecisionPointToSpike, redBackstage_ToDecisionPointTrajectory, redBackstage_SpikeToDecisionPoint;
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


        drive.setPoseEstimate(CSTB_AutoTrajectories.redBackstage_StartPos);
        CSTB_AutoTrajectories.generateTrajectories(drive);

        //gripper.close();
////////////////////////////DEFINING PARK TRAJECTORIES//////////////////////////////

        ////////////////////////////////////DONE DEFINING PARK TRAJECTORIES///////////////////////////////////////


        switch (redPropThreshold.getPropPosition()) {
            case "left":
                redBackstage_DecisionPointToSpike = CSTB_AutoTrajectories.redBackstage_DecisionPointToLeftSpike;
                redBackstage_SpikeToDecisionPoint = CSTB_AutoTrajectories.redBackstage_LeftSpikeToDecisionPoint;
                telemetry.addLine("park traj 1");
                break;
            default:
            case "center":
                redBackstage_DecisionPointToSpike = CSTB_AutoTrajectories.redBackstage_DecisionPointToCenterSpike;
                redBackstage_SpikeToDecisionPoint = CSTB_AutoTrajectories.redBackstage_CenterSpikeToDecisionPoint;
                telemetry.addLine("park traj 2");
                break;
            case "right":
                redBackstage_DecisionPointToSpike = CSTB_AutoTrajectories.redBackstage_DecisionPointToRightSpike;
                redBackstage_SpikeToDecisionPoint = CSTB_AutoTrajectories.redBackstage_RightSpikeToDecisionPoint;
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

                new CSTB_FollowTrajectoryCommand(drive, CSTB_AutoTrajectories.redBackstage_StartPositionToDecisionPoint),
                new CSTB_FollowTrajectoryCommand(drive, redBackstage_DecisionPointToSpike),
                new WaitCommand(1000),
//                new CSTB_FollowTrajectoryCommand(drive, redBackstage_SpikeToDecisionPoint),
                new CSTB_FollowTrajectoryCommand(drive, CSTB_AutoTrajectories.redBackstage_DecisionPointToMiddlePark)

        ));

    }
}



//package org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024;
//
////import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.redBackstage_ToCenterSpike;
////import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.redBackstage_ToDecisionPoint;
////import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.redBackstage_ToLeftSpike;
////import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.redBackstage_ToMiddlePark;
////import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.redBackstage_ToRightSpike;
//
//import android.util.Size;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//import org.firstinspires.ftc.teamcode.TESTBED.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.TESTBED.visionTesting.RedPropDetection;
//import org.firstinspires.ftc.vision.VisionPortal;
//
//import java.util.concurrent.TimeUnit;
//
////public class CSTB_redBackstage_ {
//
//
//@Autonomous(group = "drive", name = "CSTB red backstage PARK ")
//public class CSTB_redBackstage_Park extends CommandOpMode {
//    ElapsedTime runtime = new ElapsedTime();
//
//    private CSTB_SampleMecanumDrive drive;
//    private VisionPortal portal;
//    private RedPropDetection redPropThreshold;
//    //    private Lift lift;
//    private TrajectorySequence pixelTrajectoryRight, pixelTrajectoryMiddle, pixelTrajectoryLeft, redBackstage_parkTrajectory;
//    private FtcDashboard dashboard = FtcDashboard.getInstance();
//
//    @Override
//    public void initialize() {
////        MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        ////////‼️‼️⁉️⁉️CAMERA INITIALIZATION/DEFINING ⁉️⁉️⁉️
////        public void runOpMode () throw InterruptedException {
//        redPropThreshold = new RedPropDetection();
//        portal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
////                .addProcessor(redPropThreshold)
//                .setCameraResolution(new Size(640, 480))
////                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
////                .enableLiveView(true)
////                .setCamera()
//                .addProcessor(redPropThreshold)
//                .build();
//
//        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Camera", "Waiting");
//            telemetry.update();
//            while (!isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                sleep(20);
//            }
//            telemetry.addData("Camera", "Ready");
//            telemetry.update();
//        }
//
//        ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
//        GainControl gainControl = portal.getCameraControl(GainControl.class);
//
//        boolean wasExposureSet = exposureControl.setMode(ExposureControl.Mode.Manual);
//        exposureControl.setExposure(50, TimeUnit.MILLISECONDS);
//        gainControl.setGain(0);
//
////      everything above is included in the previous comment, this order needs to be maintained.
//
////            waitForStart();
////        }
//
//        drive = new CSTB_SampleMecanumDrive(hardwareMap);
//
//
//        drive.setPoseEstimate(CSTB_AutoTrajectories.redBackstage_StartPos);
//        CSTB_AutoTrajectories.generateTrajectories(drive);
//
//
////gripper.close();
//////////////////////////////DEFINING PARK TRAJECTORIES//////////////////////////////
//
//
//////////////////////////////////////DONE DEFINING PARK TRAJECTORIES///////////////////////////////////////
//
//
//        switch (redPropThreshold.getPropPosition()) {
//            case "left":
//                redBackstage_parkTrajectory = redBackstage_ToLeftSpike;
//                telemetry.addLine("park trajectory 1");
//                break;
//            default:
//            case "center":
//                redBackstage_parkTrajectory = redBackstage_ToCenterSpike;
//                telemetry.addLine("park trajectory 2");
//                break;
//            case "right":
//                redBackstage_parkTrajectory = redBackstage_ToRightSpike;
//                telemetry.addLine("park trajectory 3");
//                break;
//        }
//
//        telemetry.setMsTransmissionInterval(50);
//
//        while (!isStarted() && !isStopRequested()) {
//
//            telemetry.addLine("waitForStart");
//            telemetry.update();
//            sleep(20);
//        }
//
//
//        telemetry.update();
//        schedule(new SequentialCommandGroup(
//                new CSTB_FollowTrajectoryCommand(drive, redBackstage_ToDecisionPoint),
//                new CSTB_FollowTrajectoryCommand(drive, redBackstage_parkTrajectory),
//                new WaitCommand(500),
//                new CSTB_FollowTrajectoryCommand(drive, redBackstage_ToMiddlePark)
//
//        ));
//
//    }
//}