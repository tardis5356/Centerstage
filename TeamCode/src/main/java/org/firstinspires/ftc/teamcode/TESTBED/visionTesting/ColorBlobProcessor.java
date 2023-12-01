package org.firstinspires.ftc.teamcode.TESTBED.visionTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvCameraViewMode;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@TeleOp(name = "ColorBlobProcessor", group = "TeleOp")
public class ColorBlobProcessor extends LinearOpMode {
    private OpenCvCamera camera;
    private Scalar lowerRed = new Scalar(0, 0, 100);
    private Scalar upperRed = new Scalar(50, 50, 255);

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    @Override
    public void runOpMode() {
        // Initialize the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Create the blob detection pipeline
        camera.setPipeline(new RedBlobDetectionPipeline());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(camera, 10);

    }

    // Custom OpenCvPipeline for blob detection
    private class RedBlobDetectionPipeline extends OpenCvPipeline {
        private Mat workingMat = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            // Convert the input frame to HSV color space
            Imgproc.cvtColor(input, workingMat, Imgproc.COLOR_RGB2HSV);

            // Create a binary mask for red color in the HSV frame
            Core.inRange(workingMat, lowerRed, upperRed, workingMat);

            // Perform any additional image processing, filtering, or blob detection here

            // Return the processed frame
            return workingMat;
        }
    }
}