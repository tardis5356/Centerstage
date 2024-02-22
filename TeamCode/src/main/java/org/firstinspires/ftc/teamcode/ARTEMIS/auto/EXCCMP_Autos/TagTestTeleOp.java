package org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos;

import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.AutoUtils.relocalize;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.AutoUtils.relocalize2;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.RobotAlignToTagRange;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.RobotToStateCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Arm;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Intake;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.LEDs;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Lift;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Webcams;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Winch;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Wrist;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;


@TeleOp(name = "TagTestTeleop", group = "AGen1")
public class TagTestTeleOp extends CommandOpMode {
    //gamepads
    private GamepadEx driver1, driver2;

    //drivetrain motors and variables
    private DcMotorEx mFL, mFR, mBL, mBR;
    double FB, LR, Rotation;

    //keep track of time to ensure that e-game specific items dont trigger
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime rumbleTimer = new ElapsedTime();

    // speed multiplier (granny mode)
    double FAST_SPEED_MULTIPLIER = 1;
    double SLOW_SPEED_MULTIPLIER = 0.5;
    double CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER;

    public enum RobotState {
        INTAKE,
        INTAKING,
        TRANSITION,
        DEPOSIT
    }

    org.firstinspires.ftc.teamcode.ARTEMIS.teleop.Gen1_TeleOp.RobotState robotState = org.firstinspires.ftc.teamcode.ARTEMIS.teleop.Gen1_TeleOp.RobotState.INTAKE;

    public static boolean intakeStageActive = true;
    public static boolean highArmPosition = false;

    //intake and intake commands
    private Intake intake;

    //winch and winch commands
    private Winch winch;

    //lift and lift coms
    private Lift lift;

    private RobotToStateCommand robotToDepositCommand, robotToIntakeCommand, robotGrabPixelsCommand;

    //LEDs and coms
    private LEDs leds;

    //Gripper and coms
    private Gripper gripper;

    //wrist and coms
    private Wrist wrist;

    //arm and coms
    private Arm arm;

    private Drivetrain drivetrain;

    private Webcams webcams;

    public boolean aTagHomingActive = false;

    public RobotAlignToTagRange robotAlignToTagTest;

    private DcMotorEx mW;

    IMU imu;
    public static double targetTheta = 0;
    public static boolean thetaPIDActive = false;
    public static double theta_p = 1.0;
    public static double theta_i = 0;
    public static double theta_d = 0.01;
    BasicPID thetaPID = new BasicPID(new PIDCoefficients(theta_p, theta_i, theta_d));
    AngleController controller = new AngleController(thetaPID);

    private double integral = 0;
    private double prevError = 0;

//    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//    OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//    FtcDashboard.getInstance().startCameraStream(camera,

    @Override
    public void initialize() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        //init controllers
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        //init intake stuff and LEDs
        intake = new Intake(hardwareMap);
        leds = new LEDs(hardwareMap);

        //init winch stuff
        winch = new Winch(hardwareMap);

        //init lift stuff
        lift = new Lift(hardwareMap);

        //init gripper stuff
        gripper = new Gripper(hardwareMap);

        //init wrist stuff
        wrist = new Wrist(hardwareMap);

        //init arm stuff
        arm = new Arm(hardwareMap);

        drivetrain = new Drivetrain(hardwareMap);
        webcams = new Webcams(hardwareMap);

        robotToDepositCommand = new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "deposit");
        robotToIntakeCommand = new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "intake");
        robotGrabPixelsCommand = new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "grab_pixels");

        robotAlignToTagTest = new RobotAlignToTagRange(drivetrain, webcams, "back", 4, 5, 3, false);

//        imu = hardwareMap.get(IMU.class, "imuEx");
//
//        imu.initialize(
//                new IMU.Parameters(
//                        new RevHubOrientationOnRobot(
//                                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
//                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
//                        )
//                )
//        );
//
//        imu.resetYaw();

        webcams.setCamera("back");

        leds.setLEDstate("idle");

        //init and set up drive motors
        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");

        //this motor physically runs opposite. For convenience, reverse direction.
        mBR.setDirection(DcMotorSimple.Direction.REVERSE);
        mFR.setDirection(DcMotorSimple.Direction.REVERSE);

        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    // This is pretty much your while OpMode is active loop
    public void run() {
        super.run();

        double start = System.nanoTime();


        //lift always runs with manual control tied to gamepads unless stated otherwise
        lift.manualControl(cubicScaling(gamepad2.left_stick_y), gamepad2.right_stick_y);

//        double currentTheta = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

//        PIDController thetaController = new PIDController(theta_p, theta_i, theta_d);

//        telemetry.addData("theta pid active", thetaPIDActive);
//        telemetry.addData("target theta", AngleUnit.normalizeDegrees(targetTheta));
//        telemetry.addData("current theta", Math.toDegrees(AngleUnit.normalizeDegrees(currentTheta)));
//        telemetry.addData("error", Math.toDegrees(AngleUnit.normalizeDegrees(targetTheta) - Math.toRadians(currentTheta)));

        //map drive vars to inputs
        //fb is forward backward movement
        //lr is left right
        //rotation is rotation of the robot when the center of rotation is still

        FB = cubicScaling(gamepad1.left_stick_y);
        LR = cubicScaling(-gamepad1.left_stick_x) * 1.2;
//        if (thetaPIDActive) {
//            targetTheta += cubicScaling(-gamepad1.right_stick_x) * 10;// * 0.75f)*8;
////            Rotation = thetaController.calculate(AngleUnit.normalizeDegrees(currentTheta), AngleUnit.normalizeDegrees(targetTheta));
////            Rotation = calculateOutput(targetTheta, currentTheta);
//            Rotation = controller.calculate(Math.toRadians(targetTheta), currentTheta);
//            telemetry.addData("theta pid power", Rotation);
//        } else
        Rotation = cubicScaling(-gamepad1.right_stick_x);// * 0.75f);

        //map motor power to vars (tb tested)
        //depending on the wheel, forward back, left right, and rotation's power may be different
        //think, if fb is positive, thus the bot should move forward, will the motor drive the bot forward if its power is positive.
//        if (!gamepad1.a) {
        double mFLPower = FB + LR + Rotation;
        double mFRPower = FB - LR - Rotation;
        double mBLPower = FB - LR + Rotation;
        double mBRPower = FB + LR - Rotation;


        telemetry.addData("localize", relocalize(webcams.getCurrentDetections(webcams.getActiveAprilTagProcessor()), drivetrain.getYawRadians()));

        telemetry.addData("localize2", relocalize2(webcams.getCurrentDetections(webcams.getActiveAprilTagProcessor()), drivetrain.getYawRadians()));

        telemetry.addData("detections: ", webcams.getActiveAprilTagProcessor().getDetections().size());

//        if(webcams.getActiveAprilTagProcessor().getDetections().size() != 0) {
//            List<Double> x = new ArrayList<>();
//            List<Double> y = new ArrayList<>();
//            List<Double> t = new ArrayList<>();
//
//            final DecimalFormat df = new DecimalFormat("0.00");
//
////            telemetry.addData("fresh detections", webcams.getActiveAprilTagProcessor().getFreshDetections());
//            for (AprilTagDetection detection : webcams.getActiveAprilTagProcessor().getDetections()) {
//                x.add(RBYtoXYT(detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw, 180)[0] + detection.metadata.fieldPosition.get(0));
//                y.add(RBYtoXYT(detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw, 180)[0] + detection.metadata.fieldPosition.get(1));
//                Quaternion q = detection.metadata.fieldOrientation;
//                t.add(RBYtoXYT(detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw, 180)[0] + atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z)); //+2*Math.acos(detection.metadata.fieldOrientation.y)); //atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z
//                telemetry.addData("x", "%.2f", RBYtoXYT(detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw, 180)[0]); // + detection.metadata.fieldPosition.get(0)));
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
//            telemetry.addData("botPose", new Pose2d(xAvg, yAvg, Math.toRadians((drivetrain.getYaw()+tAvg)/2)));
//
////            drive.setPoseEstimate(new Pose2d(xAvg, yAvg, Math.toRadians((drivetrain.getYaw()+tAvg)/2)));
//        }

        if (aTagHomingActive) {//robotAlignToTagTest.isActive()
            telemetry.addData("target found", robotAlignToTagTest.isTargetFound());
            telemetry.addData("active camera", webcams.getActiveCamera());
            List<Integer> visibleTags = null;
            for (AprilTagDetection detection : webcams.getCurrentDetections(webcams.getActiveAprilTagProcessor())) {
//                assert visibleTags != null;
//                visibleTags.add(detection.id);
            }
//            telemetry.addData("current detections", visibleTags);
            telemetry.addData("current detections", webcams.getCurrentDetections(webcams.getActiveAprilTagProcessor()));
            if (webcams.getDesiredTag(webcams.getCurrentDetections(webcams.getActiveAprilTagProcessor()), 5) != null) {
//                telemetry.addData("getCurrentRange", robotAlignToTagTest.getCurrentRange());
//                telemetry.addData("getRangeError", robotAlignToTagTest.getRangeError());
                telemetry.addLine();
                telemetry.addData("desired tag", webcams.getDesiredTag(webcams.getCurrentDetections(webcams.getActiveAprilTagProcessor()), 5).ftcPose.range);
            }
            telemetry.addData("all powers", robotAlignToTagTest.getDriveStrafeTurnPower());
            telemetry.addLine();
            telemetry.addData("drive power", robotAlignToTagTest.getDriveStrafeTurnPower().get(0));
            telemetry.addData("strafe power", robotAlignToTagTest.getDriveStrafeTurnPower().get(1));
            telemetry.addData("turn power", robotAlignToTagTest.getDriveStrafeTurnPower().get(2));
            telemetry.addLine();
            double tagDrivePower = robotAlignToTagTest.getDriveStrafeTurnPower().get(0);
            double tagStrafePower = robotAlignToTagTest.getDriveStrafeTurnPower().get(1);
            double tagTurnPower = robotAlignToTagTest.getDriveStrafeTurnPower().get(2);

            mFL.setPower(tagDrivePower + tagStrafePower + tagTurnPower);
            mFR.setPower(tagDrivePower - tagStrafePower - tagTurnPower);
            mBL.setPower(tagDrivePower - tagStrafePower + tagTurnPower);
            mBR.setPower(tagDrivePower + tagStrafePower - tagTurnPower);
        } else {
            mFL.setPower(mFLPower * CURRENT_SPEED_MULTIPLIER);
            mFR.setPower(mFRPower * CURRENT_SPEED_MULTIPLIER);
            mBL.setPower(mBLPower * CURRENT_SPEED_MULTIPLIER);
            mBR.setPower(mBRPower * CURRENT_SPEED_MULTIPLIER);
        }

//        FtcDashboard.getInstance().startCameraStream(webcams.getBackCameraStream(), 0);

//        }
//        telemetry.addData("looptime", System.nanoTime() - start);

        telemetry.addData("\nintake power", intake.getIntakePower());

        telemetry.update();
    }

    private double cubicScaling(float joystickValue) {
        return (0.2 * joystickValue + 0.8 * Math.pow(joystickValue, 3));
    }
}



