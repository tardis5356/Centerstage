package org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends SubsystemBase {
    private Servo sArmLeft, sArmRight;

    public Arm(HardwareMap hardwareMap){
        sArmLeft = hardwareMap.get(Servo.class, "sAL");
        sArmRight = hardwareMap.get(Servo.class, "sAR");
    }

    @Override

    public void periodic(){}

    public void toIntake(){
        sArmLeft.setPosition(BotPositions.ARM_LEFT_INTAKE);
        sArmRight.setPosition(BotPositions.ARM_RIGHT_INTAKE);
    }

    public void toTransition(){
        sArmLeft.setPosition(BotPositions.ARM_LEFT_TRANSITION_POSITION);
        sArmRight.setPosition(BotPositions.ARM_RIGHT_TRANSITION_POSITION);
    }

    public void toDeposit(){
        sArmLeft.setPosition(BotPositions.ARM_LEFT_DEPOSIT);
        sArmRight.setPosition(BotPositions.ARM_RIGHT_DEPOSIT);
    }

}