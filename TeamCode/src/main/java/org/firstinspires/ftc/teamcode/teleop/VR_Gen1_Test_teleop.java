package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Intake;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands.IntakeIn;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands.IntakeOut;

@TeleOp(name="VR_Gen1_Debug")
public class VR_Gen1_Test_teleop extends CommandOpMode {
    private GamepadEx driver1;
    private GamepadEx driver2;

    private DcMotorEx mFL;
    private DcMotorEx mFR;
    private DcMotorEx mBL;
    private DcMotorEx mBR;
    double FB;
    double LR;
    double Rotation;

    private Intake intake;
    private IntakeIn intakeIn;
    private IntakeOut intakeOut;
    @Override
    public void initialize(){

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        intake = new Intake(hardwareMap);
        intakeIn = new IntakeIn(intake);
        intakeOut = new IntakeOut(intake);

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER) == true)
                .whenActive(intakeIn);

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.LEFT_BUMPER) == true)
                .whenActive(intakeOut);

        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");

        mBR.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    @Override
    public void run() {
        super.run();

        FB = gamepad1.left_stick_y;
        LR = gamepad1.left_stick_x;
        Rotation = gamepad1.right_stick_x;

        mFL.setPower(FB-LR-Rotation);
        mFR.setPower(FB+LR+Rotation);
        mBL.setPower(FB+LR-Rotation);
        mBR.setPower(FB-LR+Rotation);

        telemetry.addData("LeftStickY", FB);
        telemetry.addData("LeftStickX", LR);
        telemetry.addData("RightStickX", Rotation);
        telemetry.update();

    }
}
