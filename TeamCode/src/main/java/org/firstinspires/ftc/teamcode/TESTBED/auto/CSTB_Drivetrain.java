package org.firstinspires.ftc.teamcode.TESTBED.auto;

import com.qualcomm.robotcore.hardware.DcMotor;

public class CSTB_Drivetrain {
    DcMotor frontRight;

    DcMotor frontLeft;

    DcMotor backRight;

    DcMotor backLeft;


    public CSTB_Drivetrain(DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR){
        this.frontLeft = FL;
        this.frontRight = FR;
        this.backLeft = BL;
        this.backRight = BR;

    }

    public void CSTBDrive(double FB, double LR, double Rotation){

        this.frontLeft.setPower(FB+LR+Rotation);
        this.frontRight.setPower(FB-LR-Rotation);
        this.backLeft.setPower(FB-LR+Rotation);
        this.backRight.setPower(FB+LR-Rotation);

    }
}


