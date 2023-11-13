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
        ArmLeft.setPosition(BotPositions.LEFT_ARM_INTAKE_POSITION);
        ArmRight.setPosition(BotPositions.RIGHT_ARM_INTAKE_POSITION);
    }

    public void ArmToIntakePrep(){
        ArmLeft.setPosition(BotPositions.LEFT_ARM_INTAKE_PREP_POSITION);
        ArmRight.setPosition(BotPositions.RIGHT_ARM_INTAKE_PREP_POSITION);
    }

    public void ArmToOutPut(){
        ArmLeft.setPosition(BotPositions.LEFT_ARM_OUTPUT_POSITION);
        ArmRight.setPosition(BotPositions.RIGHT_ARM_OUTPUT_POSITION);
    }

}