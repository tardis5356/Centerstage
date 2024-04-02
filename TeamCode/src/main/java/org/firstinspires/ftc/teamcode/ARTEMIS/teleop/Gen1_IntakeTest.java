package org.firstinspires.ftc.teamcode.ARTEMIS.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.RobotAlignToTagRange;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.intakeCommands.IntakeInCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.intakeCommands.IntakeOutCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Arm;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Intake;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.LEDs;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Webcams;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Wrist;

@Config
@TeleOp(name = "Gen1_IntakeTest", group = "AGen1")
public class Gen1_IntakeTest extends CommandOpMode {
    //gamepads
    private GamepadEx driver1, driver2;

    //drivetrain motors and variables
    private DcMotorEx mFL, mFR, mBL, mBR;
    double FB, LR, Rotation;


    //intake and intake commands
    private Intake intake;
    private IntakeInCommand intakeInCommand;
    private IntakeOutCommand intakeOutCommand;

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

        //init gripper stuff
        gripper = new Gripper(hardwareMap);

        //init wrist stuff
        wrist = new Wrist(hardwareMap);

        //init arm stuff
        arm = new Arm(hardwareMap);

        drivetrain = new Drivetrain(hardwareMap);
        webcams = new Webcams(hardwareMap);

        mW = hardwareMap.get(DcMotorEx.class, "mW");

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

        leds.setLEDstate("idle");
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

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_UP))
                .whenActive(intake::downFirstPixel);
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_RIGHT))
                .whenActive(intake::downSecondPixel);
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_DOWN))
                .whenActive(intake::downThirdPixel);
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_LEFT))
                .whenActive(intake::downFourthPixel);
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.A))
                .whenActive(intake::downFifthPixel);

        //button map intake commands
        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.15 || driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.15)
                .cancelWhenActive(intakeOutCommand)
                .whenActive(intakeInCommand)
                .whenActive(new InstantCommand(() -> {
//                    intake.downTeleOp();
                }
                ));
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


        //map motor power to vars (tb tested)
        //depending on the wheel, forward back, left right, and rotation's power may be different
        //think, if fb is positive, thus the bot should move forward, will the motor drive the bot forward if its power is positive.
//        if (!gamepad1.a) {
        double mFLPower = FB + LR + Rotation;
        double mFRPower = FB - LR - Rotation;
        double mBLPower = FB - LR + Rotation;
        double mBRPower = FB + LR - Rotation;

        mFL.setPower(mFLPower * 1);
        mFR.setPower(mFRPower * 1);
        mBL.setPower(mBLPower * 1);
        mBR.setPower(mBRPower * 1);


//        FtcDashboard.getInstance().startCameraStream(webcams.getBackCameraStream(), 0);

//        }

        telemetry.addData("LeftStickY", gamepad2.left_stick_y);
        telemetry.addData("RightStickY", gamepad2.left_stick_y);
        telemetry.addData("arm distance", arm.getArmDistance());
        telemetry.addData("intake left distance", intake.getIntakeLeftDistance());
        telemetry.addData("intake right distance", intake.getIntakeRightDistance());
        telemetry.addData("inIntakeEntering", arm.inIntakeEntering());
        telemetry.addData("inIntakeExiting", arm.inIntakeExiting());


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

