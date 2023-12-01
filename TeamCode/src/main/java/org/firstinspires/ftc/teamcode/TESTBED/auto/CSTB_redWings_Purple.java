package org.firstinspires.ftc.teamcode.TESTBED.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.TESTBED.visionTesting.RedPropDetection;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

//public class CSTB_redWings_Park {
@Disabled
@Autonomous(group = "drive", name = "CSTB red wings PURPLE")
public class CSTB_redWings_Purple extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private CSTB_SampleMecanumDrive drive;
    //    private Lift lift;
    private VisionPortal portal;
    private RedPropDetection redPropThreshold;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize() {
//        MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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


//            portal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d"));
//        Starting here, the following code until line 54 needs to occur before the waitForStart in order for
//        exposure to be properly set and able to be viewed using the Driver Station preview mode
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



        drive = new CSTB_SampleMecanumDrive(hardwareMap);


        drive.setPoseEstimate(CSTB_AutoTrajectories.redWings_StartPos);
        CSTB_AutoTrajectories.generateTrajectories(drive);

        //gripper.close();

        telemetry.setMsTransmissionInterval(50);

        telemetry.update();
        schedule( new SequentialCommandGroup(

                new CSTB_FollowTrajectoryCommand(drive, CSTB_AutoTrajectories.redWings_StartPositionToDecisionPoint),
                new CSTB_FollowTrajectoryCommand(drive, CSTB_AutoTrajectories.redWings_DecisionPointToMiddlePark)
        ));

        while(!isStopRequested() && !opModeIsActive()){
            telemetry.addData("Prop Position", redPropThreshold.getPropPosition());
            telemetry.update();
            sleep(100);
        }
    }
}

