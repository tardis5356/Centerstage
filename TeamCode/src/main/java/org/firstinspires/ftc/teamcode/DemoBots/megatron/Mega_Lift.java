package org.firstinspires.ftc.teamcode.DemoBots.megatron;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Mega_Lift {

    DcMotorSimple liftMotor;

    DcMotor encoder;

    double liftPower;

    boolean liftDirectionU;
    boolean liftDirectionD;

    boolean setNow;

    int offSet = 0;

    public Mega_Lift(DcMotorSimple lM, DcMotor encoder){
        this.encoder = encoder;
        this.liftMotor = lM;
    }

    public void rise(boolean liftDirectionU, boolean liftDirectionD){

        this.liftDirectionD = liftDirectionD;
        this.liftDirectionU = liftDirectionU;



        if (this.liftDirectionU == true) {

            this.liftPower = -0.9;

        }
        else if (this.liftDirectionD == true){

            this.liftPower = 0.9;

        }
        else if (this.liftDirectionU == false && this.liftDirectionD == false){

            this.liftPower = 0;

        }

        if(this.getLiftPosition()<-715){
            this.liftPower = 0.5;
        }
//        else if(this.encoder.getCurrentPosition()<0){
//            this.liftPower = 0;
//        }


        this.liftMotor.setPower(this.liftPower/1.2);

    }

    public void setToZero(boolean setNow){

        this.setNow = setNow;

        if (setNow == true) {
            this.offSet = this.encoder.getCurrentPosition();
        }
        else{}
    }

    public int getLiftPosition(){

        return this.encoder.getCurrentPosition()-this.offSet;

    }

}
