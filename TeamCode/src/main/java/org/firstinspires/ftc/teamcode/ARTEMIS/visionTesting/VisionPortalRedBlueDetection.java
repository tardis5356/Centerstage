package org.firstinspires.ftc.teamcode.ARTEMIS.visionTesting;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.ARTEMIS.vision.BluePropDetection;
import org.firstinspires.ftc.teamcode.ARTEMIS.vision.RedPropDetection;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Vision Test")
@Disabled
public class VisionPortalRedBlueDetection extends LinearOpMode {

    private VisionPortal portal;
    private RedPropDetection redPropThreshold;
    private BluePropDetection bluePropThreshold;

    @Override
    public void runOpMode() throws InterruptedException {
        redPropThreshold = new RedPropDetection();
        bluePropThreshold = new BluePropDetection();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .enableLiveView(true)
//                .setCamera()
                .addProcessor(redPropThreshold)
                .addProcessor(bluePropThreshold)
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

//      everything above is included in the previous comment, this order needs to be maintained.

        waitForStart();

//        exposureControl.setExposure(1000, TimeUnit.MILLISECONDS);


        while (opModeIsActive()) {


//            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//                exposureControl.setMode(ExposureControl.Mode.Manual);
//                sleep(50);
//            telemetry.addData("Exposure Time", exposureControl.getExposure(TimeUnit.MILLISECONDS));
//            telemetry.addData("Exposure Min", exposureControl.getMinExposure(TimeUnit.MILLISECONDS));
//            telemetry.addData("Exposure Max", exposureControl.getMaxExposure(TimeUnit.MILLISECONDS));
//            telemetry.addData("Exposure?", wasExposureSet);
//            telemetry.addData("Gain", gainControl.getGain());
//            telemetry.addData("Gain Min", gainControl.getMinGain());
//            telemetry.addData("Gain Max", gainControl.getMaxGain());
//            telemetry.addData("Prop Position if red", redPropThreshold.getPropPosition());
            telemetry.addData("Prop Position if blue", bluePropThreshold.getPropPosition());
            telemetry.addData("Averaged Left Box Red", "%.4f", redPropThreshold.getAveragedLeftBoxRed());
            telemetry.addData("Averaged Right Box Red", "%.4f", redPropThreshold.getAveragedRightBoxRed());
            telemetry.addData("Averaged Left Box Blue", "%.6f", bluePropThreshold.getAveragedLeftBoxBlue());
            telemetry.addData("Averaged Right Box Blue", "%.6f", bluePropThreshold.getAveragedRightBoxBlue());
            telemetry.update();
        }


    }
//
    //Will output prop position on Driver Station Console


}
//}