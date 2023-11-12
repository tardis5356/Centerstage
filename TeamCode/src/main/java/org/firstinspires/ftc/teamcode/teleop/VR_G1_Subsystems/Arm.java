package org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends SubsystemBase {
    private Servo ArmLeft, ArmRight;

    public Arm(HardwareMap hardwareMap){
        ArmLeft = hardwareMap.get(Servo.class, "sAL");
        ArmRight = hardwareMap.get(Servo.class, "sAR");
    }

    @Override

    public void periodic(){}

    public void ArmToIntake(){
        ArmLeft.setPosition(.8);
        ArmRight.setPosition(.2);
    }

    public void ArmToIntakePrep(){
        ArmLeft.setPosition(.65);
        ArmRight.setPosition(.35);
    }

    public void ArmToOutPut(){
        ArmLeft.setPosition(.3);
        ArmRight.setPosition(.7);
    }

}