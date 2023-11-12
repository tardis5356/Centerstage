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
        sGL.setPosition(BotPositions.LEFT_GRIPPER_OPEN_POSITION);
    }

    public void releaseRight(){
        sGR.setPosition(BotPositions.RIGHT_GRIPPER_OPEN_POSITION);
    }

    public void grabLeft(){
        sGL.setPosition(BotPositions.LEFT_GRIPPER_CLOSED_POSITION);
    }

    public void grabRight(){
        sGR.setPosition(BotPositions.RIGHT_GRIPPER_CLOSED_POSITION);
    }
}
