package org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class Base_CSTB extends LinearOpMode {
    DcMotor mFL;

    DcMotor mFR;

    DcMotor mBL;

    DcMotor mBR;


    public void defineComponentCSTB(){

        mFR = hardwareMap.get(DcMotor.class,  "mFR");
        mFL = hardwareMap.get(DcMotor.class,  "mFL");
        mBR = hardwareMap.get(DcMotor.class,  "mBR");
        mBL = hardwareMap.get(DcMotor.class,  "mBL");

        mFL.setDirection(DcMotor.Direction.REVERSE);
        mBL.setDirection(DcMotor.Direction.REVERSE);

    }
}
