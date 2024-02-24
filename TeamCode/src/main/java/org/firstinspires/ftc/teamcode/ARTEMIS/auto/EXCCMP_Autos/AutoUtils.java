package org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos;

import static org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions.BACK_WEBCAM_X_OFFSET;
import static org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions.BACK_WEBCAM_Y_OFFSET;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    public static Pose2d relocalize6(List<AprilTagDetection> detections, double headingRad, Telemetry telemetry) {
        List<Double> x = new ArrayList<>();
        List<Double> y = new ArrayList<>();

        String numFormat = "%.2f";

        /*

        **ALL POSITIONS ARE TO CENTERS OF ELEMENTS**

        1. Bot location vs Tag
        2. Tag vs Field
        3. Bot vs Field

        TODO: make case to flip x axis when quaternion x is 0 (for audience wall tags)

         */

        // flip heading because this is an inverse transformation (coordinate system isn't rotating, bot is rotating)
        double flippedHeading = -headingRad;

        double finalX = 0;
        double finalY = 0;

        for (AprilTagDetection detection : detections) {
//            if(detection.metadata.fieldOrientation.x == 0){} // if tag is on wall


//            Pose2d tagPose = vectorFToPose2d(detection.metadata.fieldPosition);
            Pose2d tagPose = vectorFToPose2d(AutoUtils.getCenterStageTagLibrary().lookupTag(detection.metadata.id).fieldPosition);
            AprilTagPoseFtc ftcPose = detection.ftcPose;

            telemetry.addData("tag name", detection.metadata.name);

            // 1. Bot location vs Tag
            double x_camera = ftcPose.x;
            double y_camera = ftcPose.y;

            double x_botToTag = (y_camera + BACK_WEBCAM_X_OFFSET) * Math.cos(flippedHeading) - x_camera * Math.sin(flippedHeading);
            double y_botToTag = -(y_camera + BACK_WEBCAM_X_OFFSET) * Math.sin(flippedHeading) - x_camera * Math.cos(flippedHeading);

            telemetry.addData("x_botToTag", numFormat, x_botToTag);
            telemetry.addData("y_botToTag", numFormat, y_botToTag);

            // 2. Tag vs Field
            double x_tagInRR = -tagPose.getX();
            double y_tagInRR = -tagPose.getY();

            telemetry.addData("x_tagInRR", numFormat, x_tagInRR);
            telemetry.addData("y_tagInRR", numFormat, y_tagInRR);

            // 3. Bot vs Field
            double x_botToField = x_botToTag + x_tagInRR;
            double y_botToField = y_botToTag + y_tagInRR;

            telemetry.addData("x_botToField", numFormat, x_botToField);
            telemetry.addData("y_botToField", numFormat, y_botToField);

            finalX += x_botToField;
            finalY += y_botToField;

            telemetry.addLine();
        }
        if (finalX == 0)
            return null;
        else
            return new Pose2d(finalX / detections.size(), finalY / detections.size(), headingRad);
    }

    private static Pose2d vectorFToPose2d(VectorF vector) {
        return new Pose2d(vector.get(0), vector.get(1));
    }

    public static double quaternionToHeading(Quaternion q) {
        return Math.atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
    }

    // this position library credit Michael from team 14343 (@overkil on Discord)
    public static AprilTagLibrary getCenterStageTagLibrary()
    {
        return new AprilTagLibrary.Builder()
                .addTag(1, "BlueAllianceLeft",
                        2, new VectorF(61.75f, 41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(2, "BlueAllianceCenter",
                        2, new VectorF(61.75f, 35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(3, "BlueAllianceRight",
                        2, new VectorF(61.75f, 29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(4, "RedAllianceLeft",
                        2, new VectorF(61.75f, -29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(5, "RedAllianceCenter",
                        2, new VectorF(61.75f, -35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(6, "RedAllianceRight",
                        2, new VectorF(61.75f, -41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(7, "RedAudienceWallLarge",
                        5, new VectorF(-70.25f, -40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(8, "RedAudienceWallSmall",
                        2, new VectorF(-70.25f, -35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(9, "BlueAudienceWallSmall",
                        2, new VectorF(-70.25f, 35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(10, "BlueAudienceWallLarge",
                        5, new VectorF(-70.25f, 40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .build();
    }
}
