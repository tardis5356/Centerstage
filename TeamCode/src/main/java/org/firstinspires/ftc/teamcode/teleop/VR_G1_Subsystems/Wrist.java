package org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist extends SubsystemBase {

    //Create Servo objects
    private Servo ServRotate, ServTilt;

    //Map Servos to physical parts
    public Wrist (HardwareMap hardwareMap){
        ServRotate = hardwareMap.get(Servo.class, "sWGR");
        ServTilt = hardwareMap.get(Servo.class,"sWGP");
    }

    @Override
    public void periodic(){}

    public void WristToIntakePrep(){
        ServRotate.setPosition(.45);
        ServTilt.setPosition(1);
    }

    public void TiltToIntake(){
        ServTilt.setPosition(.7);
    }

    public void TiltToOutput(){
        ServTilt.setPosition(.2);
    }

    public void rotateLeft(){
        ServRotate.setPosition(.75);
    }

    public void rotateRight(){
        ServRotate.setPosition(.2);
    }

    public void rotateSquare(){
        ServRotate.setPosition(.45);
    }


}
