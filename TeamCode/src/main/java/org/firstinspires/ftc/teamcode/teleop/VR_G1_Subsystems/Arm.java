package org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Arm extends SubsystemBase {
    private Servo sArmLeft, sArmRight;

    private ColorSensor colorArm;

    public Arm(HardwareMap hardwareMap){
        sArmLeft = hardwareMap.get(Servo.class, "sAL");
        sArmRight = hardwareMap.get(Servo.class, "sAR");

        colorArm = hardwareMap.get(ColorSensor.class, "colorArm");
        colorArm.enableLed(false);
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

    public boolean inIntake() {
        if (((DistanceSensor) colorArm).getDistance(DistanceUnit.CM) <= 15)
            return true;
        else
            return false;
    }
}