package org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

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

    public static Pose2d localize(List<AprilTagDetection> detections, double headingRad) {
//        List<AprilTagDetection> detections = getAprilTagDetections();

        if (detections.size() == 0) return null;

        double xSum = 0;
        double ySum = 0;

        for (AprilTagDetection detection : detections) {
            Pose2d tagPos = vectorFToPose2d(detection.metadata.fieldPosition);
            double tagHeading = quaternionToHeading(detection.metadata.fieldOrientation);

            double x = detection.ftcPose.x - BotPositions.BACK_WEBCAM_X_OFFSET;
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

    private static Pose2d vectorFToPose2d(VectorF vector) {
        return new Pose2d(vector.get(0), vector.get(1));
    }

    public static double quaternionToHeading(Quaternion q) {
        return Math.atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    }
}
