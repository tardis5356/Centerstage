package org.firstinspires.ftc.teamcode.vision.visionTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//        import org.firstinspires.ftc.teamcode.EasyOpenCV_Examples.WebcamExample;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


//***********THIS ONE DOESN'T REALLY WORK****************

import java.util.concurrent.TimeUnit;
//
/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the skystone when lined up with
 * the sample regions over the first 3 stones.
 */
@Autonomous
public class RedBlueDetection extends LinearOpMode {
    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;
//    private VisionPortal visionPortal = null;        // Used to manage the video source.
    private static String tardisPosition = "NONE";
    private static String elementColor = "NONE";

    ExposureControl myExposureControl;

    @Override
    public void runOpMode() {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .build();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);
        //webcam.setPipeline(new TeamElementPositionTest.SkystoneDeterminationPipeline());

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //Upright rotation works, do not set to sideways left
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        // Set camera controls unless we are stopping.



        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            //   telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("avg1", pipeline.avg1);
            telemetry.addData("avg2", pipeline.avg2);
            telemetry.addData("avg3", pipeline.avg3);
            //   telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            //   telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            //   telemetry.update();
//            telemetry.addData("Anti-Oranginess", pipeline.getAnalysis());
//            telemetry.addData("Number of Rings", pipeline.getTEPosition());
            telemetry.addData("position", pipeline.getTEPosition());
            telemetry.addData("position", getPosition());
            telemetry.addData("elementColor", elementColor);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }
    /////////////////////////////////////////////////////// End of Op Mode //////////////////


    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {

        boolean viewportPaused;

        /*
         * An enum to define the team element position
         */
        public enum PowerCellPosition {
            LEFT,
            CENTER,
            RIGHT,
            NONE
        }

        public enum teamElementColor {
            RED,
            BLUE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(55, 130);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(120, 140);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(230, 140);
        static final int REGION_WIDTH = 50; //HEIGHT on screen
        static final int REGION_HEIGHT = 100; //WIDTH on screen
        // static final int FOUR_RING_THRESHOLD = 110;
        // static final int ONE_RING_THRESHOLD = 122;

        /*
         * Points which actually define the sample reg4ion rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_HEIGHT,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_WIDTH);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);


        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat region1_Cr, region2_Cr, region3_Cr;
        Mat region1_RGB, region2_RGB, region3_RGB;
        Mat YCrCb = new Mat();
        Mat Y = new Mat();
        Mat Cb = new Mat();
        Mat Cr = new Mat();

        Mat RGB = new Mat();
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile PowerCellPosition position = PowerCellPosition.NONE;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        void inputToY(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Y, 0);
        }

        void inputToCr(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cr, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */

//            inputToCb(firstFrame);
//
//            /*
//             * Submats are a persistent reference to a region of the parent
//             * buffer. Any changes to the child affect the parent, and the
//             * reverse also holds true.
//             */
//            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
//            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
//            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
//
//            inputToCr(firstFrame);
//
//            region1_Cr = Cr.submat(new Rect(region1_pointA, region1_pointB));
//            region2_Cr = Cr.submat(new Rect(region2_pointA, region2_pointB));
//            region3_Cr = Cr.submat(new Rect(region3_pointA, region3_pointB));
//
//            region1_RGB = RGB.submat(new Rect(region1_pointA, region1_pointB));
//            region2_RGB = RGB.submat(new Rect(region2_pointA, region2_pointB));
//            region3_RGB = RGB.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);


            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
//            avg1 = (int) Core.mean(region1_Cb).val[0];
//            avg2 = (int) Core.mean(region2_Cb).val[0];
//            avg3 = (int) Core.mean(region3_Cb).val[0];

             inputToCr(input);

//            avg1 = (int) Core.mean(region1_Cr).val[0];
//            avg2 = (int) Core.mean(region2_Cr).val[0];
//            avg3 = (int) Core.mean(region3_Cr).val[0];

            avg1 = (int) Core.mean(region1_RGB).val[0];
            avg2 = (int) Core.mean(region2_RGB).val[0];
            avg3 = (int) Core.mean(region3_RGB).val[0];



            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            position = PowerCellPosition.NONE;
            //ringCount = 4;
//            int maxOneTwo = Math.max(avg1, avg2);
//            int max = Math.max(maxOneTwo, avg3);
            int minOneTwo = Math.min(avg1, avg2);
            int min = Math.min(minOneTwo, avg3);

            double superMean = (avg1 + avg2 + avg3) / 3;
            double diffLeft = Math.abs(avg1 - superMean);
            double diffCenter = Math.abs(avg2 - superMean);
            double diffRight = Math.abs(avg3 - superMean);

            double maxLeftCenter = Math.max(diffLeft, diffCenter);
            double superMax = Math.max(maxLeftCenter, diffRight);


            //adding in math for mean and abs value
            //(avg1 + avg2 + avg3) / 3; - mean
            //

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
//            if (min == avg1) // Was it from region 1?
            if (superMax == diffLeft) {
                tardisPosition = "LEFT";
                position = PowerCellPosition.LEFT; // Record our analysis
                diffLeft = avg1 - superMean;
                if (diffLeft > 0) {
                    elementColor = "RED";
                } else if (diffLeft < 0) {
                    elementColor = "BLUE";
                }

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
//            } else if (min == avg2) // Was it from region 2?
            } else if (superMax == diffCenter) {
                tardisPosition = "CENTER";
                position = PowerCellPosition.CENTER; // Record our analysis
                diffCenter = avg2 - superMean;
                if (diffCenter > 0) {
                    elementColor = "RED";
                } else if (diffCenter < 0) {
                    elementColor = "BLUE";
                }

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
//            } else if (min == avg3) // Was it from region 3?
            } else if (superMax == diffRight) {
                tardisPosition = "RIGHT";
                position = PowerCellPosition.RIGHT; // Record our analysis
                diffRight = avg3 - superMean;
                if (diffRight > 0) {
                    elementColor = "RED";
                } else if (diffRight < 0) {
                    elementColor = "BLUE";
                }

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public int getAnalysis() {
            return avg1;
        }

        public PowerCellPosition getTEPosition() {
            return position;
        }

    }

    static public String getPosition() {
        return tardisPosition;
    }

}


