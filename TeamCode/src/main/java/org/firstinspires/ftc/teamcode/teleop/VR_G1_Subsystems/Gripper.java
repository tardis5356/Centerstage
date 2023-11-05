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

    public void release(){
        sGR.setPosition(.8);
        sGL.setPosition(.3);
    }

    public void grab(){
        sGR.setPosition(.2);
        sGL.setPosition(.9);
    }
}
