package org.firstinspires.ftc.teamcode.ARTEMIS.commands.autoCommands;

import static java.lang.Math.atan2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.AutoUtils;
import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.ArrayList;
import java.util.List;

public class RelocalizeFromTagCommand extends CommandBase {
    SampleMecanumDrive drive;
    AprilTagProcessor aprilTagProcessor;

    List<Double> x = new ArrayList<>();
    List<Double> y = new ArrayList<>();
    List<Double> t = new ArrayList<>();

    public RelocalizeFromTagCommand(SampleMecanumDrive drive, AprilTagProcessor aprilTagProcessor){
        this.drive = drive;
        this.aprilTagProcessor = aprilTagProcessor;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(aprilTagProcessor.getFreshDetections() != null) {
            for (AprilTagDetection detection : aprilTagProcessor.getFreshDetections()) {
                x.add(AutoUtils.RBYtoXYT(detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw, 180)[0] + detection.metadata.fieldPosition.get(0));
                y.add(AutoUtils.RBYtoXYT(detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw, 180)[0] + detection.metadata.fieldPosition.get(1));
                Quaternion q = detection.metadata.fieldOrientation;
                t.add(AutoUtils.RBYtoXYT(detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw, 180)[0] + atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z)); //+2*Math.acos(detection.metadata.fieldOrientation.y)); //atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z
            }

            double xAvg = 0;
            double yAvg = 0;
            double tAvg = 0;

            for (int i = 0; i < x.size(); i++) {
                xAvg += x.get(i);
                xAvg += y.get(i);
                xAvg += t.get(i);
            }

            xAvg /= x.size();
            yAvg /= x.size();
            tAvg /= x.size();

            drive.setPoseEstimate(new Pose2d(xAvg, yAvg, Math.toRadians(tAvg)));
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {


    }
}
