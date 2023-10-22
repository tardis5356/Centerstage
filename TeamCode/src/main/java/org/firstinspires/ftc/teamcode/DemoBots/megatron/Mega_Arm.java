package org.firstinspires.ftc.teamcode.DemoBots.megatron;

import com.qualcomm.robotcore.hardware.Servo;

public class Mega_Arm {

    Servo aS;

    double armPosition;

    public Mega_Arm(Servo a){
        this.aS = a;
    }

    public void armSet(double aPos){
        this.armPosition = aPos;
        this.aS.setPosition(this.armPosition);
    }


}
