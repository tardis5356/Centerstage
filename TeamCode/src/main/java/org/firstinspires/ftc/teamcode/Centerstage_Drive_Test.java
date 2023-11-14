package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp (name = "Centerstage_Drive_Test")
public class Centerstage_Drive_Test extends LinearOpMode {

    double FB;
    double LR;
    double Rotation;

    DcMotor mFL, mFR, mBL, mBR;

    @Override
    public void runOpMode() {

        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");


        mBR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            FB = gamepad1.left_stick_y;
            LR = -gamepad1.left_stick_x;
            Rotation = -gamepad1.right_stick_x;

            mFL.setPower(FB+LR+Rotation);
            mFR.setPower(FB-LR-Rotation);
            mBL.setPower(FB-LR+Rotation);
            mBR.setPower(FB+LR-Rotation);



        }

    }

}
