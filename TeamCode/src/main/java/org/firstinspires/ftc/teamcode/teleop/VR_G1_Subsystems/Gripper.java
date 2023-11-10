package org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Gripper extends SubsystemBase{
    private Servo sGR;
    private Servo sGL;

    public Gripper(HardwareMap hardwareMap){
        sGR = hardwareMap.get(Servo.class, "sGR");
        sGL = hardwareMap.get(Servo.class, "sGL");
    }

    @Override
    public void periodic(){}

    public void releaseLeft(){
        sGL.setPosition(.3);
    }

    public void releaseRight(){
        sGR.setPosition(.8);
    }

    public void grabLeft(){
        sGL.setPosition(.9);
    }

    public void grabRight(){
        sGR.setPosition(.2);
    }
}
