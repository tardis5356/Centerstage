package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Intake;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands.IntakeComs.IntakeIn;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands.IntakeComs.IntakeOut;

import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Winch;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands.WinchComs.DeployWinch;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands.WinchComs.PullUpBot;

import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Lift;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands.LiftToPositionCommand;


@TeleOp(name="VR_Gen1_Debug")
public class VR_Gen1_Test_teleop extends CommandOpMode {
    //gamepads
    private GamepadEx driver1;
    private GamepadEx driver2;

    //drivetrain motors and variables
    private DcMotorEx mFL;
    private DcMotorEx mFR;
    private DcMotorEx mBL;
    private DcMotorEx mBR;
    double FB;
    double LR;
    double Rotation;

    //intake and intake commands
    private Intake intake;
    private IntakeIn intakeIn;
    private IntakeOut intakeOut;

    //winch and winch commands
    private Winch winch;
    private DeployWinch deployWinch;
    private PullUpBot pullUpBot;

    //lift and lift coms
    private Lift lift;
    private LiftToPositionCommand liftToIntakePositionCommand;


    @Override
    public void initialize(){

        //init controllers
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        //init intake stuff
        intake = new Intake(hardwareMap);
        intakeIn = new IntakeIn(intake);
        intakeOut = new IntakeOut(intake);

        //init winch stuff
        winch = new Winch(hardwareMap);
        deployWinch = new DeployWinch(winch);
        pullUpBot = new PullUpBot(winch);

        //init lift stuff
        lift = new Lift(hardwareMap);
        liftToIntakePositionCommand = new LiftToPositionCommand(lift, -10, 20);

        //button map intake commands
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER) == true)
                .whenActive(intakeIn);

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.LEFT_BUMPER) == true)
                .whenActive(intakeOut);

        //button map winch commands
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_UP) == true)
                .whenActive(deployWinch);

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_DOWN) == true)
                .whenActive(pullUpBot);

        //map buttons to lift positions
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_DOWN) == true)
                .whenActive(liftToIntakePositionCommand);

        //init and set up drive motors
        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");

        mBR.setDirection(DcMotorSimple.Direction.REVERSE);


    }
    @Override
    public void run() {
        super.run();
        lift.manualControl(-gamepad2.left_stick_y, -gamepad2.right_stick_y);


        //map drive vars to inputs
        FB = gamepad1.left_stick_y;
        LR = gamepad1.left_stick_x;
        Rotation = gamepad1.right_stick_x;

        //map motor power to vars (tb tested)
        mFL.setPower(FB-LR-Rotation);
        mFR.setPower(FB+LR+Rotation);
        mBL.setPower(FB+LR-Rotation);
        mBR.setPower(FB-LR+Rotation);

        //telem
        telemetry.addData("LeftStickY", FB);
        telemetry.addData("LeftStickX", LR);
        telemetry.addData("RightStickX", Rotation);
        telemetry.update();

    }
}
