package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.IntakeInCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeOutCommand;
import org.firstinspires.ftc.teamcode.commands.LiftToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.RobotToStateCommand;
import org.firstinspires.ftc.teamcode.commands.WinchDeployCommand;
import org.firstinspires.ftc.teamcode.commands.WinchPullUpCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LEDs;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Winch;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;


@TeleOp(name = "Gen1 TeleOp")
public class Gen1_TeleOp extends CommandOpMode {
    //gamepads
    private GamepadEx driver1, driver2;

    //drivetrain motors and variables
    private DcMotorEx mFL, mFR, mBL, mBR;
    double FB, LR, Rotation;

    //drone launcher servo
    private Servo sDroneLauncher;

    //keep track of time to ensure that e-game specific items dont trigger
    ElapsedTime gametime = new ElapsedTime();

    // speed multiplier (granny mode)
    double FAST_SPEED_MULTIPLIER = 1.0;
    double SLOW_SPEED_MULTIPLIER = 0.5;
    double CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER;


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

    private DcMotorEx mW;

    @Override
    public void initialize() {
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

        //init lift stuff
        lift = new Lift(hardwareMap);
        liftToIntakePositionCommand = new LiftToPositionCommand(lift, -10, 20);

        //init gripper stuff
        gripper = new Gripper(hardwareMap);

        //init wrist stuff
        wrist = new Wrist(hardwareMap);

        //init arm stuff
        arm = new Arm(hardwareMap);

        mW = hardwareMap.get(DcMotorEx.class, "mW");

        robotToDepositCommand = new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "deposit");
        robotToIntakeCommand = new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "intake");
        robotGrabPixelsCommand = new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "grab_pixels");
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


        //button map intake commands
        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5 || driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(intakeInCommand);
        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5 || driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(intakeInCommand);

        // map position commands
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.LEFT_BUMPER) || driver2.getButton(GamepadKeys.Button.LEFT_BUMPER))
                .whenActive(robotToIntakeCommand);
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER) || driver2.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                .whenActive(robotToDepositCommand);

        //button map winch commands
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_UP))
                .whenActive(winchDeployCommand);
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_DOWN))
                .toggleWhenActive(winchPullUpCommand, new InstantCommand(winch::stopWinch));
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_RIGHT))
                .toggleWhenActive(() -> mW.setPower(-BotPositions.WINCH_MOTOR_POWER), () -> mW.setPower(0));
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_LEFT))
                .toggleWhenActive(new InstantCommand(winch::extendBraces), new InstantCommand(winch::overextendBraces));

        // ORIGINALLY new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_LEFT))
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.BACK))
                .toggleWhenActive((() -> sDroneLauncher.setPosition(BotPositions.DRONE_LATCHED)), () -> sDroneLauncher.setPosition(BotPositions.DRONE_UNLATCHED));

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
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.Y) || driver2.getButton(GamepadKeys.Button.Y))
                .toggleWhenActive(
                        new ParallelCommandGroup(
                                new InstantCommand(gripper::grabRight),
                                new InstantCommand(gripper::grabLeft)
                        ),
                        new ParallelCommandGroup(
                                new InstantCommand(gripper::releaseRight),
                                new InstantCommand(gripper::releaseLeft)
                        )
                );

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.B))
                .toggleWhenActive(new InstantCommand(gripper::grabRight), new InstantCommand(gripper::releaseRight));
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.X))
                .toggleWhenActive(new InstantCommand(gripper::grabLeft), new InstantCommand(gripper::releaseLeft));

        // Speed controls
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.B))
                .whenActive(() -> CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER);
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.X))
                .whenActive(() -> CURRENT_SPEED_MULTIPLIER = SLOW_SPEED_MULTIPLIER);

//        new Trigger(() -> leds.checkLeftPixel() == true)
//                .whenActive(new SequentialCommandGroup(
//                        new WaitCommand(750),
//                        new InstantCommand(gripper::grabLeft)
//                        ));
//
//        new Trigger(()-> leds.checkRightPixel() == true)
//                .whenActive(new SequentialCommandGroup(
//                        new WaitCommand(750),
//                        new InstantCommand(gripper::grabRight)
//                ));

        // automatically grab pixels
        new Trigger(() -> leds.checkLeftPixel() && leds.checkRightPixel())
                .whenActive(new SequentialCommandGroup(
                        new WaitCommand(750),
                        robotGrabPixelsCommand,
//                        new WaitCommand(2000),
                        intakeOutCommand
                ));

        //triggers to roll wrist
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_RIGHT))
                .whenActive(new InstantCommand(wrist::rollToRight));
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_LEFT))
                .whenActive(new InstantCommand(wrist::rollToLeft));
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_UP))
                .whenActive(new InstantCommand(wrist::rollToCentered));


        //init and set up drive motors
        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");

        //this motor physically runs opposite. For convenience, reverse direction.
        mBR.setDirection(DcMotorSimple.Direction.REVERSE);
        mFR.setDirection(DcMotorSimple.Direction.REVERSE);

        //init the drone servo
        sDroneLauncher = hardwareMap.get(Servo.class, "sDL");
    }

    @Override
    // This is pretty much your while OpMode is active loop
    public void run() {
        super.run();

        //lift always runs with manual control tied to gamepads unless stated otherwise
        lift.manualControl(gamepad2.left_stick_y, gamepad2.right_stick_y);


        //map drive vars to inputs
        //fb is forward backward movement
        //lr is left right
        //rotation is rotation of the robot when the center of rotation is still
        FB = gamepad1.left_stick_y;
        LR = -gamepad1.left_stick_x;
        Rotation = -gamepad1.right_stick_x;

        //map motor power to vars (tb tested)
        //depending on the wheel, forward back, left right, and rotation's power may be different
        //think, if fb is positive, thus the bot should move forward, will the motor drive the bot forward if its power is positive.
        mFL.setPower(FB + LR + Rotation);
        mFR.setPower(FB - LR - Rotation);
        mBL.setPower(FB - LR + Rotation);
        mBR.setPower(FB + LR - Rotation);

//        if (gamepad1.a) { //&& gametime.seconds() > 90){
//            sDroneLauncher.setPosition(BotPositions.DRONE_UNLATCHED);
//        } else {
//            sDroneLauncher.setPosition(BotPositions.DRONE_LATCHED);
//        }

//        if (gamepad1.dpad_left)
//            mW.setPower(-BotPositions.WINCH_MOTOR_POWER);

        //telemetry stoof
//        telemetry.addData("LeftStickY", FB);
//        telemetry.addData("LeftStickX", LR);
//        telemetry.addData("RightStickX", Rotation);

        telemetry.addData("LeftStickY", gamepad2.left_stick_y);
        telemetry.addData("arm distance", arm.getArmDistance());
        telemetry.addData("intake left distance", intake.getIntakeLeftDistance());
        telemetry.addData("intake right distance", intake.getIntakeRightDistance());
        telemetry.addData("touchLift", lift.getLiftBase());
        telemetry.addData("touchLift", lift.getLiftTargetPosition());
        telemetry.addData("inIntakeEntering", arm.inIntakeEntering());
        telemetry.addData("inIntakeExiting", arm.inIntakeExiting());
        telemetry.addData("inIntake", lift.manualActive);
        telemetry.addData("liftPower", lift.getLiftPower());

        telemetry.update();

    }
}
