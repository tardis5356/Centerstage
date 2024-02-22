package org.firstinspires.ftc.teamcode.ARTEMIS.commands.autoCommands;

import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.AutoUtils.relocalize;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    Telemetry telemetry;

    public RelocalizeFromTagCommand(SampleMecanumDrive drive, Drivetrain drivetrain, AprilTagProcessor aprilTagProcessor, Telemetry telemetry) {
        this.drive = drive;
        this.drivetrain = drivetrain;
        this.aprilTagProcessor = aprilTagProcessor;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Pose2d newPose = relocalize(aprilTagProcessor.getDetections(), drivetrain.getYawRadians());
//        if (newPose != null) {
//            drive.setPoseEstimate(newPose);
//            telemetry.addData("relocalized using apriltags ", newPose);
//        } else {
            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians((drivetrain.getYawDegrees() + 360) % 360)));
//            telemetry.addData("relocalized using imu ", (drivetrain.getYawDegrees() + 360) % 360);
//        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {


    }
}

