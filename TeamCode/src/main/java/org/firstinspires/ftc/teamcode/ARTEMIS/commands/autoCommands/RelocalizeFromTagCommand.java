package org.firstinspires.ftc.teamcode.ARTEMIS.commands.autoCommands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Drivetrain;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.ArrayList;
import java.util.List;

public class RelocalizeFromTagCommand extends CommandBase {
    SampleMecanumDrive drive;
    Drivetrain drivetrain;
    AprilTagProcessor aprilTagProcessor;

    List<Double> x = new ArrayList<>();
    List<Double> y = new ArrayList<>();
    List<Double> t = new ArrayList<>();

    public RelocalizeFromTagCommand(SampleMecanumDrive drive, Drivetrain drivetrain, AprilTagProcessor aprilTagProcessor) {
        this.drive = drive;
        this.drivetrain = drivetrain;
        this.aprilTagProcessor = aprilTagProcessor;
    }

    public Pose2d localize(List<AprilTagDetection> detections, double headingRad) {
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

    private Pose2d vectorFToPose2d(VectorF vector) {
        return new Pose2d(vector.get(0), vector.get(1));
    }

    public static double quaternionToHeading(Quaternion q) {
        return Math.atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
//        if (aprilTagProcessor.getFreshDetections() != null) {
//            for (AprilTagDetection detection : aprilTagProcessor.getFreshDetections()) {
//                x.add(AutoUtils.RBYtoXYT(detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw, 180)[0] + detection.metadata.fieldPosition.get(0));
//                y.add(AutoUtils.RBYtoXYT(detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw, 180)[0] + detection.metadata.fieldPosition.get(1));
////                Quaternion q = detection.metadata.fieldOrientation;
//                t.add(AutoUtils.RBYtoXYT(detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw, 180)[0] + quaternionToHeading(detection.metadata.fieldOrientation)); //+2*Math.acos(detection.metadata.fieldOrientation.y)); //Math.atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z
//            }
//
//            double xAvg = 0;
//            double yAvg = 0;
//            double tAvg = 0;
//
//            for (int i = 0; i < x.size(); i++) {
//                xAvg += x.get(i);
//                xAvg += y.get(i);
//                xAvg += t.get(i);
//            }
//
//            xAvg /= x.size();
//            yAvg /= x.size();
//            tAvg /= x.size();
//
//            drive.setPoseEstimate(new Pose2d(xAvg, yAvg, Math.toRadians((drivetrain.getYaw() + tAvg) / 2)));
//        }
        Pose2d newPose = localize(aprilTagProcessor.getDetections(), drivetrain.getYawRadians());
        if (newPose != null)
            drive.setPoseEstimate(newPose);
        else
            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drivetrain.getYawDegrees()));
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {


    }
}

