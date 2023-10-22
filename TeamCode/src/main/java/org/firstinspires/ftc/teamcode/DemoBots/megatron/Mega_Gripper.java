package org.firstinspires.ftc.teamcode.DemoBots.megatron;

import com.qualcomm.robotcore.hardware.CRServo;

public class Mega_Gripper {

    CRServo leftBand;
    CRServo rightBand;

    double lBPower;
    double rBPower;

    public Mega_Gripper(CRServo lB, CRServo rB){

        this.leftBand = lB;
        this.rightBand = rB;

    }

    public void Mega_Grip(double lPower, double rPower){

        this.lBPower = lPower;
        this.rBPower = rPower;

        this.leftBand.setPower(this.lBPower);
        this.rightBand.setPower(this.rBPower);

    }


}
