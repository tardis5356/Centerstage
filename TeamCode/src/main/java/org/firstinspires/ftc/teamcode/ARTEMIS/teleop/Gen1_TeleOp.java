package org.firstinspires.ftc.teamcode.ARTEMIS.teleop;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.intakeCommands.IntakeInCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.intakeCommands.IntakeOutCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.LiftToPositionCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.RobotAlignToTagRange;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.RobotToStateCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.WinchDeployCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.WinchPullUpCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Arm;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Intake;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.LEDs;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Lift;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Webcams;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Winch;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Wrist;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Config
@TeleOp(name = "Gen1_TeleOp", group = "AGen1")
public class Gen1_TeleOp extends CommandOpMode {
    //gamepads
    private GamepadEx driver1, driver2;

    //drivetrain motors and variables
    private DcMotorEx mFL, mFR, mBL, mBR;
    double FB, LR, Rotation;

    //drone launcher servo
    private Servo sDroneLauncher;

    private Servo wristRoll;

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

    RobotState robotState = RobotState.INTAKE;

    public static boolean intakeStageActive = true;
    public static boolean highArmPosition = false;

    //intake and intake commands
    private Intake intake;
    private IntakeInCommand intakeInCommand;
    private IntakeOutCommand intakeOutCommand;

    //winch and winch commands
    private Winch winch;
    private WinchDeployCommand winchDeployCommand;
    private WinchPullUpCommand winchPullUpCommand;

    //lift and lift coms
    private Lift lift;
    private LiftToPositionCommand liftToIntakePositionCommand;

    private RobotToStateCommand robotToDepositCommand, robotToIntakeCommand, robotGrabPixelsCommand;

    //LEDs and coms
    private LEDs leds;

    //Gripper and coms
    private Gripper gripper;

    //wrist and coms
    private Wrist wrist;

    //arm and coms
    private Arm arm;

    private DroneLauncher launcher;

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
        intakeInCommand = new IntakeInCommand(intake, leds);
        intakeOutCommand = new IntakeOutCommand(intake);

        //init winch stuff
        winch = new Winch(hardwareMap);
        winchDeployCommand = new WinchDeployCommand(winch);
        winchPullUpCommand = new WinchPullUpCommand(winch);

        lift = new Lift(hardwareMap);
        liftToIntakePositionCommand = new LiftToPositionCommand(lift, -10, 20);

        gripper = new Gripper(hardwareMap);
        wrist = new Wrist(hardwareMap);
        arm = new Arm(hardwareMap);
        launcher = new DroneLauncher(hardwareMap);

        drivetrain = new Drivetrain(hardwareMap);
        webcams = new Webcams(hardwareMap);

        mW = hardwareMap.get(DcMotorEx.class, "mW");

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

        if (arm.inIntakeEntering()) {
            arm.toIntake();
            wrist.tiltToIntake();
        }
        wrist.rollToCentered();
        intake.up();
        intake.disableLEDs();
        winch.retractBraces();
        launcher.latch();

        leds.setLEDstate("idle");
        gamepad1.setLedColor(230, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        gamepad2.setLedColor(230, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
/*

    Driver
        Trigger Left - intake in.
        Bumper Left - intake position.

        Trigger Right -  intake out.
        Bumper Right - deposit position.

        DPAD Up - deploy winch (HOLD).
        DPAD Right - unwind winch.
        DPAD Down - pull up robot.
        DPAD Left - deploy drone (HOLD).

        Y - toggle both grippers.
        A - square to backdrop
        B - fast
        X - slow

        Start - reinit imu

    Manipulator
        Trigger Left - intake in.
        Bumper Left - intake position.

        Trigger Right -  intake out.
        Bumper Right - deposit position.

        DPAD Up -
        DPAD Right - tilt intake right.
        DPAD Down -
        DPAD Left - tilt intake left.

        Y - toggle both grippers.
        A -
        B - toggle right gripper.
        X - toggle left gripper.

 */

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.BACK))
                .whenActive(launcher::unlatch)
                .whenInactive(launcher::latch);

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON))
                .whenActive(robotAlignToTagTest)
                .whenActive(() -> {
                    aTagHomingActive = true;
                })
                .whenInactive(() -> aTagHomingActive = false);

        //button map intake commands
        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.15 || driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.15)
                .cancelWhenActive(intakeOutCommand)
                .whenActive(intakeInCommand)
                .whenActive(new InstantCommand(() -> {
                    if (intakeStageActive)
                        intake.downTeleOp();
                }));
        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.15 || driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.15)
                .cancelWhenActive(intakeInCommand)
                .whenActive(intakeOutCommand);
//                .whenActive(
//                        new ParallelCommandGroup(
//                                new InstantCommand(() -> {
////                                    robotState = RobotState.INTAKE;
//                                })
//                        )
//                );
//                .whenActive(
//                        new SequentialCommandGroup(
//                                new InstantCommand(intake::out),
//                                new WaitCommand(1500),
//                                new InstantCommand(intake::stop)
//                        )
//                );


        // map position commands
        new Trigger(() -> (driver1.getButton(GamepadKeys.Button.LEFT_BUMPER) || driver2.getButton(GamepadKeys.Button.LEFT_BUMPER)))// && (robotState != RobotState.INTAKE && robotState != RobotState.INTAKING))
                .whenActive(
//                        new SequentialCommandGroup(
                        robotToIntakeCommand
//                                new InstantCommand(() -> {
//                                    robotState = RobotState.INTAKE;
//                                })
//                        )
                )
                .whenActive(new InstantCommand(() -> {
                    robotState = RobotState.INTAKE;
                }))
                .cancelWhenActive(robotGrabPixelsCommand)
                .cancelWhenActive(robotToDepositCommand);
        new Trigger(() -> (driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER) || driver2.getButton(GamepadKeys.Button.RIGHT_BUMPER)))// && robotState != RobotState.DEPOSIT)
                .whenActive(robotToDepositCommand)
                .whenActive(new InstantCommand(() -> {
                    robotState = RobotState.DEPOSIT;
                }))
                .cancelWhenActive(robotGrabPixelsCommand)
                .cancelWhenActive(robotToIntakeCommand);

        //button map winch commands
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_UP))
                .whenActive(new InstantCommand(winch::extendBraces));
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_DOWN))
                .toggleWhenActive(winchPullUpCommand, new InstantCommand(winch::stopWinch));
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_RIGHT))
                .toggleWhenActive(() -> mW.setPower(-BotPositions.WINCH_MOTOR_POWER), () -> mW.setPower(0)); //, () -> mW.setPower(BotPositions.WINCH_MOTOR_POWER)
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_LEFT))
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(winch::overextendBraces)//,
//                        new WaitCommand(1000),
//                        new InstantCommand(winch::disablePWM)
                ));

        // ORIGINALLY new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_LEFT))
//        new Trigger(() -> driver1.getButton(GamepadKeys.Button.BACK))
//                .toggleWhenActive((() -> sDroneLauncher.setPosition(BotPositions.DRONE_LATCHED)), () -> sDroneLauncher.setPosition(BotPositions.DRONE_UNLATCHED));

        //map buttons to lift positions
//        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_DOWN))
//                .whenActive(liftToIntakePositionCommand);


        //triggers to change the led color to signal to human player what pixels are needed
        /*
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.A))
                .whenActive(new InstantCommand(() -> {
                    leds.changeColor(leds.Green);
                }));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.X))
                .whenActive(new InstantCommand(() -> {
                    leds.changeColor(leds.Purple);
                }));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.Y))
                .whenActive(new InstantCommand(() -> {
                    leds.changeColor(leds.Yellow);
                }));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.B))
                .whenActive(new InstantCommand(() -> {
                    leds.changeColor(leds.White);
                }));*/


        //triggers to open and close gripper
//        new Trigger(() -> driver1.getButton(GamepadKeys.Button.Y) || driver2.getButton(GamepadKeys.Button.Y))
//                .toggleWhenActive(
//                        new ParallelCommandGroup(
//                                new InstantCommand(gripper::grabRight),
//                                new InstantCommand(gripper::grabLeft)
//                        ),
//                        new ParallelCommandGroup(
//                                new InstantCommand(gripper::releaseRight),
//                                new InstantCommand(gripper::releaseLeft)
//                        )
//                );

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.Y) || driver2.getButton(GamepadKeys.Button.Y))
                .whenActive(
                        new ParallelCommandGroup(
                                new InstantCommand(gripper::releaseRight),
                                new InstantCommand(gripper::releaseLeft)
                        )
                );
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.A) || driver2.getButton(GamepadKeys.Button.A))
                .whenActive(
                        new ParallelCommandGroup(
                                new InstantCommand(gripper::grabRight),
                                new InstantCommand(gripper::grabLeft)
                        )
                );


        new Trigger(() -> driver2.getButton(GamepadKeys.Button.X))
                .toggleWhenActive(new InstantCommand(gripper::grabRight), new InstantCommand(gripper::releaseRight));
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.B))
                .toggleWhenActive(new InstantCommand(gripper::grabLeft), new InstantCommand(gripper::releaseLeft));

        // square to backdrop
//        new Trigger(() -> driver1.getButton(GamepadKeys.Button.A))
//                .whileActiveContinuous(() -> lift.squareToBackdropPID());

        // Speed controls
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.B))
                .whenActive(() -> CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER)
                .whenActive(new InstantCommand());
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.X))
                .whenActive(() -> CURRENT_SPEED_MULTIPLIER = SLOW_SPEED_MULTIPLIER);
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.BACK))
                .toggleWhenActive(() -> {
                    BotPositions.ARM_HIGH_POSITION = true;
                    arm.toDeposit();
                    wrist.tiltToDeposit();
                }, () -> {
                    BotPositions.ARM_HIGH_POSITION = false;
                    arm.toDeposit();
                    wrist.tiltToDeposit();
                });

        new Trigger(() -> leds.checkLeftPixel() == true)
                .whenActive(new SequentialCommandGroup(
                        new WaitCommand(250),
                        new InstantCommand(gripper::grabLeft)
                ));

        new Trigger(() -> leds.checkRightPixel() == true)
                .whenActive(new SequentialCommandGroup(
                        new WaitCommand(250),
                        new InstantCommand(gripper::grabRight)
                ));

        // automatically grab pixels
        new Trigger(() -> leds.checkLeftPixel() && leds.checkRightPixel())
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            robotState = RobotState.TRANSITION;
                            leds.setLEDstate("green");
                        }),
                        new WaitCommand(600),
                        new InstantCommand(intake::out),
                        new InstantCommand(intake::up),
//                        new WaitCommand(100),
                        new InstantCommand(arm::toTransition),
                        new InstantCommand(wrist::toTransition),
                        new WaitCommand(100),
                        new InstantCommand(() -> {
                            leds.setLEDstate("purple");
                            gamepad1.rumbleBlips(2);
                            gamepad2.rumbleBlips(2);
                        }),
                        new WaitCommand(1000),
                        new InstantCommand(intake::stop)
                ))
                .cancelWhenActive(robotToDepositCommand);

        //triggers to roll wrist
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_RIGHT))
                .whenActive(new InstantCommand(wrist::rollToRight60));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_LEFT))
                .whenActive(new InstantCommand(wrist::rollToLeft60));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_UP))
                .whenActive(new InstantCommand(wrist::rollToCentered));


        //init and set up drive motors
        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");
        wristRoll = hardwareMap.get(Servo.class, "sWGR");

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

        if (rumbleTimer.seconds() >= 0.5) {
//            if (leds.checkLeftPixel() && !leds.checkRightPixel() && !gamepad1.isRumbling() || leds.checkRightPixel() && !leds.checkLeftPixel() && !gamepad1.isRumbling()) {
//                gamepad1.rumbleBlips(1);
//                gamepad2.rumbleBlips(1);
//            }

            if (leds.checkLeftPixel() && leds.checkRightPixel()) {
                gamepad1.rumbleBlips(2);
                gamepad2.rumbleBlips(2);
            }
            rumbleTimer.reset();
        }

        //map motor power to vars (tb tested)
        //depending on the wheel, forward back, left right, and rotation's power may be different
        //think, if fb is positive, thus the bot should move forward, will the motor drive the bot forward if its power is positive.
//        if (!gamepad1.a) {
        double mFLPower = FB + LR + Rotation;
        double mFRPower = FB - LR - Rotation;
        double mBLPower = FB - LR + Rotation;
        double mBRPower = FB + LR - Rotation;

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

//        if (gamepad1.back) { //&& gametime.seconds() > 90){
//            launcher.unlatch();
//        } else {
//            sDroneLauncher.setPosition(BotPositions.DRONE_LATCHED);
//        }

        telemetry.addData("RobotState\n", robotState);

        telemetry.addData("LeftStickY", gamepad2.left_stick_y);
        telemetry.addData("RightStickY", gamepad2.left_stick_y);
        telemetry.addData("arm distance", arm.getArmDistance());
        telemetry.addData("intake left distance", intake.getIntakeLeftDistance());
        telemetry.addData("intake right distance", intake.getIntakeRightDistance());
        telemetry.addData("touchLift", lift.getLiftBase());
        telemetry.addData("touchLift", lift.getLiftTargetPosition());
        telemetry.addData("inIntakeEntering", arm.inIntakeEntering());
        telemetry.addData("inIntakeExiting", arm.inIntakeExiting());
        telemetry.addData("lift manual active", lift.manualActive);
        telemetry.addData("liftPower", lift.getLiftPower());

        telemetry.addData("\nlift target", lift.getLiftTargetPosition());
        telemetry.addData("lift position", lift.getLiftPosition());
        telemetry.addData("lift power", lift.getLiftPower());
        telemetry.addData("pid out", lift.getLiftPID());

//        telemetry.addData("distanceBackLeft", lift.getDistanceBackLeft());
//        telemetry.addData("distanceBackRight", lift.getDistanceBackRight());
//        telemetry.addData("drivePowerLeft", lift.getLeftDrivePower());
//        telemetry.addData("drivePowerRight", lift.getRightDrivePower());
//        telemetry.addData("drivePowerLeftValue", lift.getLeftDrivePowerValue());
//        telemetry.addData("drivePowerRightValue", lift.getRightDrivePowerValue());
//        telemetry.addData("leftError", lift.getLeftError());
//        telemetry.addData("rightError", lift.getRightError());

        telemetry.addData("looptime", System.nanoTime() - start);

        telemetry.addData("\nintake power", intake.getIntakePower());

        telemetry.update();
    }

    private double cubicScaling(float joystickValue) {
        return (0.2 * joystickValue + 0.8 * Math.pow(joystickValue, 3));
    }

//    public PIDController(double kp, double ki, double kd) {
//        kp = kp;
//        ki = ki;
//        kd = kd;
//    }

//    public double turnPID(double setpoint, double currentAngle) {
//        double error = (setpoint, currentAngle);
//
//        integral += error;
//        double derivative = error - prevError;
//
//        double output = (theta_p * error) + (theta_i * integral) + (theta_d * derivative);
//
//        prevError = error;
//
//        return output;
//    }

}

