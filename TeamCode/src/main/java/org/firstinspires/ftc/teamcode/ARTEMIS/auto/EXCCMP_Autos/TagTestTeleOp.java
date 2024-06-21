package org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.AutoUtils.relocalize6;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.AutoUtils.relocalizeFrontCamera;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;
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

    private SampleMecanumDrive drive;

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
        drive = new SampleMecanumDrive(hardwareMap);

        //init controllers
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        //init intake stuff and LEDs
        intake = new Intake(hardwareMap);
        leds = new LEDs(hardwareMap, gripper);

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
//        drivetrain.setStartingError();
        drivetrain.setStartingOffsetDegs(270);

        webcams.setCamera("back");

        leds.setLEDstate("idle");

//        //init and set up drive motors
//        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
//        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
//        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
//        mBR = hardwareMap.get(DcMotorEx.class, "mBR");
//
//        //this motor physically runs opposite. For convenience, reverse direction.
//        mBR.setDirection(DcMotorSimple.Direction.REVERSE);
//        mFR.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    // This is pretty much your while OpMode is active loop
    public void run() {
        super.run();

        double start = System.nanoTime();

        //lift always runs with manual control tied to gamepads unless stated otherwise
        lift.manualControl(cubicScaling(gamepad2.left_stick_y), gamepad2.right_stick_y);

        FB = cubicScaling(gamepad1.left_stick_y);
        LR = cubicScaling(-gamepad1.left_stick_x) * 1.2;
        Rotation = cubicScaling(-gamepad1.right_stick_x);// * 0.75f);

//        double mFLPower = FB + LR + Rotation;
//        double mFRPower = FB - LR - Rotation;
//        double mBLPower = FB - LR + Rotation;
//        double mBRPower = FB + LR - Rotation;

        telemetry.addData("imu yaw", drivetrain.getYawDegrees());

//        telemetry.addData("localize5", relocalize5(webcams.getCurrentDetections(webcams.getActiveAprilTagProcessor()), drivetrain.getYawRadians(), telemetry));

        telemetry.addData("# of detections: ", webcams.getActiveAprilTagProcessor().getDetections().size());


        Pose2d newPose = relocalize6(webcams.getCurrentDetections(webcams.getActiveAprilTagProcessor()), drivetrain.getYawRadians(), telemetry);
        if (newPose != null) {
            drive.setPoseEstimate(newPose);
//            telemetry.addData("relocalized using apriltags ", newPose);
        } else {
            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toRadians((drivetrain.getYawDegrees() + 360) % 360)));
//            telemetry.addData("relocalized using imu ", (drivetrain.getYawDegrees() + 360) % 360);
        }

//            mFL.setPower(mFLPower * CURRENT_SPEED_MULTIPLIER);
//            mFR.setPower(mFRPower * CURRENT_SPEED_MULTIPLIER);
//            mBL.setPower(mBLPower * CURRENT_SPEED_MULTIPLIER);
//            mBR.setPower(mBRPower * CURRENT_SPEED_MULTIPLIER);

        drive.update();

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }

    private double cubicScaling(float joystickValue) {
        return (0.2 * joystickValue + 0.8 * Math.pow(joystickValue, 3));
    }
}



