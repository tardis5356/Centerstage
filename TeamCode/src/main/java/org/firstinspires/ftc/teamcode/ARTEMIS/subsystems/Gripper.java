package org.firstinspires.ftc.teamcode.ARTEMIS.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
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
        sGL.setPosition(BotPositions.GRIPPER_LEFT_OPEN);
    }

    public void releaseRight(){
        sGR.setPosition(BotPositions.GRIPPER_RIGHT_OPEN);
    }

    public void grabLeft(){
        sGL.setPosition(BotPositions.GRIPPER_LEFT_CLOSED);
    }

    public void grabRight(){
        sGR.setPosition(BotPositions.GRIPPER_RIGHT_CLOSED);
    }
}
