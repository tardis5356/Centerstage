package org.firstinspires.ftc.teamcode.DemoBots.megatron;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Mega_drivetrain {

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;

//    double fRPower;
//    double fLPower;
//    double bRPower;
//    double bLPower;

    public Mega_drivetrain(DcMotor fL, DcMotor fR, DcMotor bL, DcMotor bR){
        this.frontRight = fR;
        this.frontLeft = fL;
        this.backRight = bR;
        this.backLeft = bL;

    }
    public void Mega_drive(double FB, double LR, double Rotation) {

        this.frontLeft.setPower((FB+LR+Rotation));
        this.frontRight.setPower(FB-LR-Rotation);
        this.backLeft.setPower(FB-LR+Rotation);
        this.backRight.setPower(FB+LR-Rotation);

    }
}
