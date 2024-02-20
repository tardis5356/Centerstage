package org.firstinspires.ftc.teamcode.ARTEMIS.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Webcams;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class RobotAlignToTagRange extends CommandBase {
    Drivetrain drivetrain;
    Webcams webcam;
    // Adjust these numbers to suit your robot.
    private double DESIRED_DISTANCE = 12; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.035  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)  // 0.28
    final double STRAFE_GAIN =  0.035 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0) // 0.28
    final double TURN_GAIN   =  0.03  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0) // 0.2

    final double MAX_AUTO_SPEED = 0.2;   //  Clip the approach speed to this max value (adjust for your robot) 0.5
    final double MAX_AUTO_STRAFE= 0.2;   //  Clip the approach speed to this max value (adjust for your robot) 0.5
    final double MAX_AUTO_TURN  = 0.2;   //  Clip the turn speed to this max value (adjust for your robot) 0.3

    private static String activeWebcam = "back";  // 1 front, 2 back
    private static int DESIRED_TAG_ID = 0;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    double drive, strafe, turn;
    double previousDrive, previousStrafe, previousTurn;

    double tolerance;

    double rangeError = 0, headingError = 0, yawError = 0;

    boolean homingActive = false;

    boolean auto = false;

    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
//    double  drive           = 0;        // Desired forward power/speed (-1 to +1)
//    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
//    double  turn            = 0;        // Desired turning power/speed (-1 to +1)

    public RobotAlignToTagRange(Drivetrain drivetrain, Webcams webcams, String active_camera, double desired_range, int target_tag, double tolerance, boolean auto) {
        this.drivetrain = drivetrain;
        this.webcam = webcams;
        this.activeWebcam = active_camera;
        this.DESIRED_DISTANCE = desired_range;
        this.DESIRED_TAG_ID = target_tag;
        this.tolerance = tolerance;
        this.auto = auto;
    }

    @Override
    public void initialize() { // runs once

        // Initialize the Apriltag Detection process

        webcam.setCamera(activeWebcam);
        homingActive = true;
//        try {
//            webcam.setManualExposure(6, 250, activeWebcam);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }

//        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
//        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
//        telemetry.addData(">", "Touch Play to start OpMode");
//        telemetry.update();
//        waitForStart();


    }

    @Override
    public void execute() { // runs continuously
        desiredTag = null;
        desiredTag = webcam.getDesiredTag(webcam.getCurrentDetections(webcam.getActiveAprilTagProcessor()), DESIRED_TAG_ID);

        if(desiredTag != null) {
            rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            headingError = desiredTag.ftcPose.bearing;
            yawError = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            previousDrive = drive;
            previousTurn = turn;
            previousStrafe = strafe;

            if (activeWebcam != "front") {
                turn *= 1;
                strafe *= -1;
            }

            targetFound = true;

        } else {
//            drive = -previousDrive;
//            turn = -previousTurn;
//            strafe = -previousStrafe;
            targetFound = false;

            drive = 0;
            turn = 0;
            strafe = 0;
//            drivetrain.driveRobot(0, 0, 0);
        }
        if(auto)
            drivetrain.driveRobot(drive, strafe, turn);
    }

    public boolean isTargetFound(){
        return targetFound;
    }

    public List<Double> getDriveStrafeTurnPower() {
        return Arrays.asList(drive, strafe, turn);
    }

    @Override
    public boolean isFinished() { // returns true when finished
//        return Math.abs(lift.getLiftPosition() - targetPosition) < tolerance;
        return Math.abs(rangeError) + Math.abs(yawError * 10) + Math.abs(headingError * 10) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
//        lift.stop();
        homingActive = false;
    }

    public boolean isActive(){
        return homingActive;
    }
}