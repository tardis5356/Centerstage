package org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024;

import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.redWings_ToDecisionPoint;
import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.redWings_ToLeftSpike;
import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.redWings_ToMiddlePark;
import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.redWings_ToRightSpike;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
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
    TrajectorySequence purpleTrajectoryRight, purpleTrajectoryMiddle, purpleTrajectoryLeft, parkTrajectory;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize() {
//        MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ////////‼️‼️⁉️⁉️CAMERA INITIALIZATION/DEFINING ⁉️⁉️⁉️
        public void runOpMode() throws InterruptedException {
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

            waitForStart();
        }

        drive = new CSTB_SampleMecanumDrive(hardwareMap);


        drive.setPoseEstimate(CSTB_AutoTrajectories.redWings_StartPos);
        CSTB_AutoTrajectories.generateTrajectories(drive);

        //gripper.close();
////////////////////////////DEFINING PARK TRAJECTORIES//////////////////////////////
        purpleTrajectoryLeft = drive.trajectorySequenceBuilder(CSTB_AutoTrajectories.redWings_StartPos)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(60, 16, Math.toRadians(90)), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                        CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                .build();

        purpleTrajectoryMiddle = drive.trajectorySequenceBuilder(CSTB_AutoTrajectories.redWings_StartPos)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(40, 12, Math.toRadians(270)), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                        CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                .build();

        purpleTrajectoryRight = drive.trajectorySequenceBuilder(CSTB_AutoTrajectories.redWings_StartPos)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(12, 16, Math.toRadians(270)), CSTB_SampleMecanumDrive.getVelocityConstraint(86, CSTB_DriveConstants.MAX_ANG_VEL, CSTB_DriveConstants.TRACK_WIDTH),
                        CSTB_SampleMecanumDrive.getAccelerationConstraint(CSTB_DriveConstants.MAX_ACCEL))
                .build();
        ////////////////////////////////////DONE DEFINING PARK TRAJECTORIES///////////////////////////////////////

        if (runtime.seconds() > 28) {
            if (redPropThreshold) {
                CommandScheduler.getInstance().cancelAll();
                switch () {
                    case 1:
                        parkTrajectory = new redWings_ToLeftSpike(drive, purpleTrajectoryLeft);
                        telemetry.addLine("park traj 1");
                        break;
                    default:
                    case 2:
                        parkTrajectory = new redWings_ToDecisionPoint(drive, purpleTrajectoryMiddle);
                        telemetry.addLine("park traj 2");
                        break;
                    case 3:
                        parkTrajectory = new redWings_ToRightSpike(drive, purpleTrajectoryRight);
                        telemetry.addLine("park traj 3");
                        break;

                    telemetry.setMsTransmissionInterval(50);

                    while (!isStarted() && !isStopRequested()) {

                        telemetry.addLine("waitForStart");
                        telemetry.update();
                        sleep(20);
                    }


                    telemetry.update();
                    schedule(new SequentialCommandGroup(

                            new CSTB_FollowTrajectoryCommand(drive, redWings_ToDecisionPoint),
                            new CSTB_FollowTrajectoryCommand(drive, redWings_ToMiddlePark)

                    ));

                }
            }}}}

