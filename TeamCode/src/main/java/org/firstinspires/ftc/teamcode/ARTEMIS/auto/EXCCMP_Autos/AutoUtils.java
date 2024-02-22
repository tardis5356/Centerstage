package org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos;

import static org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions.BACK_WEBCAM_X_OFFSET;
import static org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions.BACK_WEBCAM_Y_OFFSET;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.ArrayList;
import java.util.List;

public class AutoUtils {
    public static double[] XYTTtoRBY(double x, double y, double botTheta, double tagTheta){
        double range = Math.sqrt(x * x + y * y);
        double bearing = Math.toDegrees(Math.atan2(-x,y));
        double yaw = botTheta - tagTheta;
        return new double[]{range, bearing, yaw};
    }

    public static double[] RBYtoXYT(double range, double bearing, double yaw, double tagTheta){
        double x = -range*Math.sin(Math.toRadians(bearing));
        double y = range*Math.cos(Math.toRadians(bearing));
        double t = yaw + tagTheta;
        return new double[]{x, y, t};
    }

    public static Pose2d relocalize(List<AprilTagDetection> detections, double headingRad) {
//        List<AprilTagDetection> detections = getAprilTagDetections();

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

    public static Pose2d relocalize2(List<AprilTagDetection> detections, double headingRad){
        List<Double> x = new ArrayList<>();
        List<Double> y = new ArrayList<>();
        List<Double> t = new ArrayList<>();

        if (detections != null) {
            for (AprilTagDetection detection : detections) {
//                x.add(AutoUtils.RBYtoXYT(detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw, 180)[0] + detection.metadata.fieldPosition.get(0));
//                y.add(AutoUtils.RBYtoXYT(detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw, 180)[0] + detection.metadata.fieldPosition.get(1));
////                Quaternion q = detection.metadata.fieldOrientation;
//                t.add(AutoUtils.RBYtoXYT(detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw, 180)[0] + quaternionToHeading(detection.metadata.fieldOrientation)); //+2*Math.acos(detection.metadata.fieldOrientation.y)); //Math.atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z
                x.add(getFCPosition(detection, headingRad).getX());
                y.add(getFCPosition(detection, headingRad).getY());
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
//            return new Pose2d(x.get(0), y.get(0), (headingRad));
        }
        return null;
    }

    /**
     * @param botheading In Radians.
     * @return FC Pose of bot.
     */
    public static Pose2d getFCPosition(AprilTagDetection detection, double botheading) {
        AprilTagLibrary tags = AprilTagGameDatabase.getCurrentGameTagLibrary();

        // get coordinates of the robot in RC coordinates
        // ensure offsets are RC
        double x = detection.ftcPose.x-BACK_WEBCAM_X_OFFSET;
        double y = detection.ftcPose.y-BACK_WEBCAM_Y_OFFSET;

        // invert heading to correct properly
        botheading = -botheading;

        // rotate RC coordinates to be field-centric
        double x2 = x*Math.cos(botheading)+y*Math.sin(botheading);
        double y2 = x*-Math.sin(botheading)+y*Math.cos(botheading);

        // add FC coordinates to apriltag position
        // tags is just the CS apriltag library
        VectorF tagpose = tags.lookupTag(detection.id).fieldPosition;
        return new Pose2d(tagpose.get(0)+y2,tagpose.get(1)-x2, -botheading); // new Rotation2d(-botheading)
    }

    private static Pose2d vectorFToPose2d(VectorF vector) {
        return new Pose2d(vector.get(0), vector.get(1));
    }

    public static double quaternionToHeading(Quaternion q) {
        return Math.atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    }
}
