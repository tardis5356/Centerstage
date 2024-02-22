package org.firstinspires.ftc.teamcode.ARTEMIS.subsystems;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ARTEMIS.visionTesting.BluePropDetection;
import org.firstinspires.ftc.teamcode.ARTEMIS.visionTesting.RedPropDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import java.util.concurrent.atomic.AtomicReference;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;


public class Webcams extends SubsystemBase {
    private VisionPortal visionPortal, visionPortalBack;               // Used to manage the video source.
    private AprilTagProcessor aprilTagProcessor, aprilTagBack;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    private BluePropDetection bluePropThreshold;
    private RedPropDetection redPropThreshold;

    private CameraStreamProcessor frontCameraStream, backCameraStream;

    public String activeWebcam = "back", activeProcessor = "apriltag";

    public WebcamName frontWebcam, backWebcam;

    public Webcams(HardwareMap hardwareMap) {
        // Create the AprilTag processor by using a builder.
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        bluePropThreshold = new BluePropDetection();
        redPropThreshold = new RedPropDetection();
//        aprilTagBack = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTagProcessor.setDecimation(2);
//        aprilTagBack.setDecimation(2);

        frontWebcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        backWebcam = hardwareMap.get(WebcamName.class, "Webcam 2");

        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(frontWebcam, backWebcam);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCamera(switchableCamera)
                .addProcessors(aprilTagProcessor, redPropThreshold, bluePropThreshold)
                .setAutoStopLiveView(true)
                .enableLiveView(true)
//                .addProcessor(frontCameraStream)
                .build();

//        visionPortalBack = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
//                .addProcessor(aprilTagBack)
//                .setAutoStopLiveView(true)
//                .enableLiveView(true)
////                .addProcessor(backCameraStream)
//                .build();
    }

    public void initialize() throws InterruptedException {
//        try {
//            setManualExposure(6, 250, activeWebcam);  // Use low exposure time to reduce motion blur
//        } catch (InterruptedException e) {
//
//        }
    }

    @Override
    public void periodic() {

    }


    public AprilTagProcessor getAprilTagFrontProcessor() {
        return aprilTagProcessor;
    }


    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }

    public CameraStreamProcessor getFrontCameraStream() {
        return frontCameraStream;
    }

    public CameraStreamProcessor getBackCameraStream() {
        return backCameraStream;
    }

    public AprilTagProcessor getActiveAprilTagProcessor() {
        return getAprilTagFrontProcessor();
    }

    public List<AprilTagDetection> getCurrentDetections(AprilTagProcessor aprilTagProcessor) {
        return aprilTagProcessor.getDetections();
    }

    public void setActiveProcessor(String processor){
        if(processor == "apriltag") {
            visionPortal.setProcessorEnabled(aprilTagProcessor, true);
            visionPortal.setProcessorEnabled(bluePropThreshold, false);
            visionPortal.setProcessorEnabled(redPropThreshold, false);
            activeProcessor = "apriltag";
        } else if(processor == "redProp") {
            visionPortal.setProcessorEnabled(aprilTagProcessor, false);
            visionPortal.setProcessorEnabled(bluePropThreshold, false);
            visionPortal.setProcessorEnabled(redPropThreshold, true);
            activeProcessor = "redProp";
        } else if(processor == "blueProp") {
            visionPortal.setProcessorEnabled(aprilTagProcessor, false);
            visionPortal.setProcessorEnabled(bluePropThreshold, true);
            visionPortal.setProcessorEnabled(redPropThreshold, false);
            activeProcessor = "blueProp";
        }
    }

    public String getPropPosition(){
        if(activeProcessor == "redProp"){
            return redPropThreshold.getPropPosition();
        } else if(activeProcessor == "blueProp"){
            return bluePropThreshold.getPropPosition();
        }
        return null;
    }

    public AprilTagDetection getDesiredTag(List<AprilTagDetection> currentDetections, int desiredTagID) {
        boolean targetFound;
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((desiredTagID < 0) || (detection.id == desiredTagID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
//                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                return null;
                // This tag is NOT in the library, so we don't have enough information to track to it.
//                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        return desiredTag;
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    public void setManualExposure(int exposureMS, int gain, String webcam) throws
            InterruptedException {
        // Wait for the camera to be open, then use the controls

        VisionPortal activePortal;

        activePortal = visionPortal;
        if (activePortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (activePortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Camera", "Waiting");
//            telemetry.update();
//            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                sleep(20);
//            }
//            telemetry.addData("Camera", "Ready");
//            telemetry.update();
        }

//            try {
//                // Set camera controls unless we are stopping.
//                sleep(50);
//                ExposureControl exposureControl = activePortal.getCameraControl(ExposureControl.class);
//                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//                    exposureControl.setMode(ExposureControl.Mode.Manual);
//                    sleep(50);
//                }
//                exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
//                sleep(20);
//                GainControl gainControl = activePortal.getCameraControl(GainControl.class);
//                gainControl.setGain(gain);
//                sleep(20);//add exception to method signature
//            } catch (InterruptedException e) {
//
//            }
    }

    public void setCamera(String webcam) {
        if (webcam.toLowerCase() == "front") {
            activeWebcam = "front";
            visionPortal.setActiveCamera(frontWebcam);
//            visionPortal.resumeStreaming();
        } else {
            // back webcam
            activeWebcam = "back";
            visionPortal.setActiveCamera(backWebcam);
//            visionPortal.resumeStreaming();
        }
    }

    public String getActiveCamera() {
        return activeWebcam;
    }
}