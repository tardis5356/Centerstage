package org.firstinspires.ftc.teamcode.ARTEMIS.visionTesting;


import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class RedPropDetection implements VisionProcessor {

    Mat testMatRed = new Mat();
    Mat highMatRed = new Mat();
    Mat lowMatRed = new Mat();
    Mat finalMatRed = new Mat();
    Mat testMatBlue = new Mat();
    Mat highMatBlue = new Mat();
    Mat lowMatBlue = new Mat();
    Mat finalMatBlue = new Mat();
    double redThreshold = 0.04;
    double blueThreshold = 0.0010;
    double averagedLeftBoxRed;
    double averagedRightBoxRed;
    double averagedLeftBoxBlue;
    double averagedRightBoxBlue;

    String outStr = "left"; //Set a default value in case vision does not work

    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    static final Rect LEFT_RECTANGLE = new Rect( // 640 x 480 (X by Y)
            new Point(0, 250), //anchor (upper left corner)
            new Point(250, 400) //width, height
    );
    static final Rect RIGHT_RECTANGLE = new Rect(
            new Point(450, 250),
            new Point(640, 400)
    );

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
//        inputToCb(firstFrame);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, testMatRed, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(frame, testMatBlue, Imgproc.COLOR_RGB2HSV);


        //notes: In OpenCV, Hue has values from 0 to 180, Saturation and Value from 0 to 255.
        // Thus, OpenCV uses HSV ranges between (0-180, 0-255, 0-255). In OpenCV, the H values 179,
        // 178, 177 and so on are as close to the true RED as H value 1, 2, 3 and so on.

        //RED!!
        Scalar lowHSVRedLower = new Scalar(0, 100, 20);  //Beginning of Color Wheel
        Scalar lowHSVRedUpper = new Scalar(10, 255, 255);

        Scalar redHSVRedLower = new Scalar(160, 100, 20); //Wraps around Color Wheel
        Scalar highHSVRedUpper = new Scalar(180, 255, 255);

        Core.inRange(testMatRed, lowHSVRedLower, lowHSVRedUpper, lowMatRed);
        Core.inRange(testMatRed, redHSVRedLower, highHSVRedUpper, highMatRed);

        testMatRed.release();

        Core.bitwise_or(lowMatRed, highMatRed, finalMatRed);

        lowMatRed.release();
        highMatRed.release();

        double leftBoxRed = Core.sumElems(finalMatRed.submat(LEFT_RECTANGLE)).val[0];
        double rightBoxRed = Core.sumElems(finalMatRed.submat(RIGHT_RECTANGLE)).val[0];

        averagedLeftBoxRed = leftBoxRed / LEFT_RECTANGLE.area() / 255;
        averagedRightBoxRed = rightBoxRed / RIGHT_RECTANGLE.area() / 255; //Makes value [0,1]

//        //BLUE!!
        Scalar lowHSVBlueLower = new Scalar(105, 100, 20);  //Beginning of Color Wheel
        Scalar lowHSVBlueUpper = new Scalar(135, 255, 255);

//        Scalar blueHSVBLueLower = new Scalar(40, 100, 20); //Wraps around Color Wheel
//        Scalar highHSVBlueUpper = new Scalar(55, 255, 255);
//
        Core.inRange(testMatBlue, lowHSVBlueLower, lowHSVBlueUpper, finalMatBlue);
//        Core.inRange(testMatBlue, blueHSVBLueLower, highHSVBlueUpper, highMatBlue);
//
        testMatBlue.release();

//        Core.bitwise_or(lowMatBlue, highMatBlue, finalMatBlue);

//        lowMatBlue.release();
//        highMatBlue.release();

        double leftBoxBlue = Core.sumElems(finalMatBlue.submat(LEFT_RECTANGLE)).val[0];
        double rightBoxBlue = Core.sumElems(finalMatBlue.submat(RIGHT_RECTANGLE)).val[0];

        averagedLeftBoxBlue = leftBoxBlue / LEFT_RECTANGLE.area() / 255;
        averagedRightBoxBlue = rightBoxBlue / RIGHT_RECTANGLE.area() / 255; //Makes value [0,1]


        if (averagedLeftBoxRed > redThreshold) {        //Must Tune Red Threshold
            outStr = "left";
        } else if (averagedRightBoxRed > redThreshold) {
            outStr = "right";
        } else {
            outStr = "center";

//        if (averagedLeftBoxRed > redThreshold) {        //Must Tune Red Threshold
//            outStr = "left";
//        } else if (averagedRightBoxRed > redThreshold) {
//            outStr = "right";
//        } else {
//            outStr = "center";
        }


//        finalMat.copyTo(frame); /*This line should only be added in when you want to see your custom pipeline
//                                  on the driver station stream, do not use this permanently in your code as
//                                  you use the "frame" mat for all of your pipelines, such as April Tags*/
        Imgproc.rectangle(
                frame, // Buffer to draw on
                new Point(LEFT_RECTANGLE.x, LEFT_RECTANGLE.y), // First point which defines the rectangle
                new Point(LEFT_RECTANGLE.x + LEFT_RECTANGLE.width, LEFT_RECTANGLE.y + LEFT_RECTANGLE.height), // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Supposed to be center block; we don't need it because it's just everything that isn't already drawn
//         */
//        Imgproc.rectangle(
//                frame, // Buffer to draw on
//                region2_pointA, // First point which defines the rectangle
//                region2_pointB, // Second point which defines the rectangle
//                BLUE, // The color the rectangle is drawn in
//                2); // Thickness of the rectangle lines
        /*
         * Draw a rectangle showing sample region 3 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                frame, // Buffer to draw on
                new Point(RIGHT_RECTANGLE.x, RIGHT_RECTANGLE.y), // First point which defines the rectangle
                new Point(RIGHT_RECTANGLE.x + RIGHT_RECTANGLE.width, RIGHT_RECTANGLE.y + RIGHT_RECTANGLE.height), // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

//        Imgproc.rectangle(
//                frame, // Buffer to draw on
//                new Point(LEFT_RECTANGLE.x, LEFT_RECTANGLE.y), // First point which defines the rectangle
//                new Point(LEFT_RECTANGLE.width, LEFT_RECTANGLE.height), // Second point which defines the rectangle
//                BLUE, // The color the rectangle is drawn in
//                2); // Thickness of the rectangle lines

        return null;            //You do not return the original mat anymore, instead return null


    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }


    public String getPropPosition() {  //Returns postion of the prop in a String
        return outStr;
    }

    public double getAveragedLeftBoxRed() {  //Returns postion of the prop in a String
        return averagedLeftBoxRed;
    }

    public double getAveragedRightBoxRed() {  //Returns postion of the prop in a String
        return averagedRightBoxRed;
    }

    public double getAveragedLeftBoxBlue() {  //Returns postion of the prop in a String
        return averagedLeftBoxBlue;
    }

    public double getAveragedRightBoxBlue() {  //Returns postion of the prop in a String
        return averagedRightBoxBlue;
    }
}

