package org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos;

import static org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions.BACK_WEBCAM_X_OFFSET;
import static org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions.BACK_WEBCAM_Y_OFFSET;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.DemoBots.primus.Pose;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class AutoUtils {
    public static double[] XYTTtoRBY(double x, double y, double botTheta, double tagTheta) {
        double range = Math.sqrt(x * x + y * y);
        double bearing = Math.toDegrees(Math.atan2(-x, y));
        double yaw = botTheta - tagTheta;
        return new double[]{range, bearing, yaw};
    }

    public static double[] RBYtoXYT(double range, double bearing, double yaw, double tagTheta) {
        double x = -range * Math.sin(Math.toRadians(bearing));
        double y = range * Math.cos(Math.toRadians(bearing));
        double t = yaw + tagTheta;
        return new double[]{x, y, t};
    }

    /**
     * tardis
     *
     * @param detections
     * @param headingRad
     * @return
     */

    public static Pose2d relocalize(List<AprilTagDetection> detections, double headingRad) {
        List<Double> x = new ArrayList<>();
        List<Double> y = new ArrayList<>();
        List<Double> t = new ArrayList<>();

        if (detections != null) {
            for (AprilTagDetection detection : detections) {
                x.add(AutoUtils.RBYtoXYT(detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw, 180)[0] + detection.metadata.fieldPosition.get(0));
                y.add(AutoUtils.RBYtoXYT(detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw, 180)[0] + detection.metadata.fieldPosition.get(1));
                t.add(AutoUtils.RBYtoXYT(detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw, 180)[0] + quaternionToHeading(detection.metadata.fieldOrientation)); //+2*Math.acos(detection.metadata.fieldOrientation.y)); //Math.atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z
            }

            double xAvg = 0;
            double yAvg = 0;
            double tAvg = 0;

            for (int i = 0; i < detections.size(); i++) {
                xAvg += x.get(i);
                yAvg += y.get(i);
//                tAvg += t.get(i);
            }

            xAvg /= x.size();
            yAvg /= x.size();
//            tAvg /= x.size();

            return new Pose2d(xAvg, yAvg, (headingRad));
        }
        return null;
    }

    /**
     * vector
     *
     * @param detections
     * @param headingRad
     * @return
     */

    public static Pose2d relocalize2(List<AprilTagDetection> detections, double headingRad) {
        if (detections.size() == 0) return null;

        double xSum = 0;
        double ySum = 0;

        for (AprilTagDetection detection : detections) {
            Pose2d tagPos = vectorFToPose2d(detection.metadata.fieldPosition);
            double tagHeading = quaternionToHeading(detection.metadata.fieldOrientation);

            double x = detection.ftcPose.x - BACK_WEBCAM_X_OFFSET;
            double y = detection.ftcPose.y - BotPositions.BACK_WEBCAM_Y_OFFSET;

//            headingRad = -headingRad;

            double x2 = x * Math.cos(headingRad) + y * Math.sin(headingRad);
            double y2 = x * -Math.sin(headingRad) + y * Math.cos(headingRad);

            double absX = tagPos.getX() - y2;
            double absY = tagPos.getY() + x2;

            xSum += absX;
            ySum += absY;
        }

        return new Pose2d(xSum / detections.size(), ySum / detections.size(), headingRad);
    }

    /**
     * using escapve velocity
     *
     * @param detections
     * @param headingRad
     * @return
     */

    public static Pose2d relocalize3(List<AprilTagDetection> detections, double headingRad) {
        List<Double> x = new ArrayList<>();
        List<Double> y = new ArrayList<>();
        List<Double> t = new ArrayList<>();

        if (detections != null) {
            for (AprilTagDetection detection : detections) {
                x.add(getFCPosition(detection, headingRad).getX());
                y.add(getFCPosition(detection, headingRad).getY());
            }

            double xAvg = 0;
            double yAvg = 0;
            double tAvg = 0;

            for (int i = 0; i < detections.size(); i++) {
                xAvg += x.get(i);
                yAvg += y.get(i);
            }

            xAvg /= x.size();
            yAvg /= x.size();

            return new Pose2d(xAvg, yAvg, (headingRad));
//            return new Pose2d(x.get(0), y.get(0), (headingRad));
        }
        return null;
    }

    /**
     * escape velocity
     */
    public static Pose2d getFCPosition(AprilTagDetection detection, double botheading) { //heading in radians
        AprilTagLibrary tags = AprilTagGameDatabase.getCurrentGameTagLibrary();

        // get coordinates of the robot in RC coordinates
        // ensure offsets are RC
        double x = detection.ftcPose.x - BACK_WEBCAM_X_OFFSET;
        double y = detection.ftcPose.y - BACK_WEBCAM_Y_OFFSET;

        // invert heading to correct properly
        botheading = -botheading;

        // rotate RC coordinates to be field-centric
        double x2 = x * Math.cos(botheading) + y * Math.sin(botheading);
        double y2 = x * -Math.sin(botheading) + y * Math.cos(botheading);

        // add FC coordinates to apriltag position
        // tags is just the CS apriltag library
        VectorF tagpose = tags.lookupTag(detection.id).fieldPosition;
        return new Pose2d(tagpose.get(0) + y2, tagpose.get(1) - x2, -botheading); // new Rotation2d(-botheading)
    }


    public static Pose2d relocalize4(List<AprilTagDetection> detections, double headingRad, Telemetry telemetry) {
        List<Double> x = new ArrayList<>();
        List<Double> y = new ArrayList<>();

        for (AprilTagDetection detection : detections) {
            /*
            x is backdrop to wing, y is alliance to alliance

            blue wing (red alliance side) is (+,+) on rr coordinate system
            blue backstage is (+,+) on ftc coordinate system

            bot heading must be flipped

            after calculations, both x and y signs need to be flipped to transpose from ftc to rr coord systems
            */

            Pose2d tagPose = vectorFToPose2d(detection.metadata.fieldPosition);
            AprilTagPoseFtc ftcPose = detection.ftcPose;

            // get hypoteneuse of bot center to atag center (not the same as camera center + offset to atag center)
            double hyp = Math.sqrt(
                    (ftcPose.x + BACK_WEBCAM_X_OFFSET) * (ftcPose.x + BACK_WEBCAM_X_OFFSET) +
                            (ftcPose.y + BACK_WEBCAM_Y_OFFSET) * (ftcPose.y + BACK_WEBCAM_Y_OFFSET));

            // trig to get bot x-y position of bot relative to atag
            double x_botToTag = hyp * Math.cos(-headingRad);
            double y_botToTag = hyp * Math.sin(-headingRad);

            // still in ftc coordinate system, get position of bot relative to atag
            double newBotX = tagPose.getX() - x_botToTag;
            double newBotY = tagPose.getY() - y_botToTag;

            telemetry.addData("Tag Heading Rad: ", Math.toRadians(quaternionToHeading(detection.metadata.fieldOrientation)));
            telemetry.addData("Tag Heading Deg: ", (quaternionToHeading(detection.metadata.fieldOrientation)));

            telemetry.addData("hyp", hyp);
            telemetry.addData("tagX", tagPose.getX());
            telemetry.addData("x_botToTag", x_botToTag);
            telemetry.addData("y_botToTag", y_botToTag);
            telemetry.addData("newBotX", newBotX);
            telemetry.addData("newBotY", newBotY);

            x.add(newBotX);
            y.add(newBotY);
        }

        double xVal = 0;
        double yVal = 0;

//        if (detections.size() % 2 == 0) {
        for (int i = 0; i < detections.size(); i++) {
            xVal += x.get(i);
            yVal += y.get(i);
        }
        xVal /= detections.size();
        yVal /= detections.size();
//        } else {
//            ArrayList<Double> sortedX = new ArrayList<>(x);
//
//            // sort the duplicate list
//            Collections.sort(sortedX);
//
//            // find the median value
//            int middleIndex1 = sortedX.size() / 2 - 1;
//            int middleIndex2 = sortedX.size() / 2;
//            double medianValue = (sortedX.get(middleIndex1) + sortedX.get(middleIndex2)) / 2.0;
//
//            // find the index of the median value in the original list
//            int medianIndex = x.indexOf(medianValue);
//
//            // get the median values
//            xVal = x.get(medianIndex);
//            yVal = y.get(medianIndex);
//        }

        // convert to rr coords and take avg of all calculated positions
        double xRR = -xVal;
        double yRR = -yVal;

        return new Pose2d(xRR, yRR, headingRad);
    }


    public static Pose2d relocalize5(List<AprilTagDetection> detections, double headingRad, Telemetry telemetry) {
        List<Double> x = new ArrayList<>();
        List<Double> y = new ArrayList<>();

        /*

        **ALL POSITIONS ARE TO CENTERS OF ELEMENTS**

        1. Calculate camera vs. tag
        2. Calculate bot vs camera
        3. Calculate bot vs tag
        4. Get tag vs field
        5. Calculate bot vs field

        TODO: fix bad sign on rotation, make case to flip x axis when quaternion x is 0 (for audience wall tags)

         */

        double finalX = 0;
        double finalY = 0;

//        headingRad = -headingRad;

        // 2. Calculate bot vs camera
        double x_botToCamera = -BACK_WEBCAM_X_OFFSET*Math.cos(headingRad);
        double y_botToCamera = BACK_WEBCAM_X_OFFSET*Math.sin(headingRad); //removed negative on beginning here

        telemetry.addData("x_botToCamera", x_botToCamera);
        telemetry.addData("y_botToCamera", y_botToCamera);

        for (AprilTagDetection detection : detections) {
            if(detection.metadata.fieldOrientation.x == 0){ // if tag is on wall

            }

            Pose2d tagPose = vectorFToPose2d(detection.metadata.fieldPosition);
            AprilTagPoseFtc ftcPose = detection.ftcPose;

            // 1. Calculate camera vs. tag
            double x_cameraToTag = ftcPose.x;
            double y_cameraToTag = ftcPose.y;

            telemetry.addData("x_cameraToTag", x_cameraToTag);
            telemetry.addData("y_cameraToTag", y_cameraToTag);

            // 3. Calculate bot vs tag
            // y-tag is outwards from tag, but x-bot is outwards from camera
            double x_botToTag = y_cameraToTag + -x_botToCamera;
            double y_botToTag = x_cameraToTag + y_botToCamera;

            telemetry.addData("x_botToTag", x_botToTag);
            telemetry.addData("y_botToTag", y_botToTag);

            // 4. Get tag vs field
            double x_tagInRR = -tagPose.getX();
            double y_tagInRR = -tagPose.getY();

            telemetry.addData("x_tagInRR", x_tagInRR);
            telemetry.addData("y_tagInRR", y_tagInRR);

            // 5. Calculate bot vs field
            double x_botToField = x_botToTag + x_tagInRR;
            double y_botToField = y_botToTag + y_tagInRR;

            telemetry.addData("x_botToField", x_botToField);
            telemetry.addData("y_botToField", y_botToField);

            finalX = x_botToField;
            finalY = y_botToField;
        }
        return new Pose2d(finalX, finalY);
    }

    private static Pose2d vectorFToPose2d(VectorF vector) {
        return new Pose2d(vector.get(0), vector.get(1));
    }

    public static double quaternionToHeading(Quaternion q) {
        return Math.atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
    }
}
