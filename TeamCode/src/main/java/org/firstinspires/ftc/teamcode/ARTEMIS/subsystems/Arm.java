package org.firstinspires.ftc.teamcode.ARTEMIS.subsystems;

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

    public void periodic(){
        colorArm.enableLed(false);
    }

    public void toIntake(){
        sArmLeft.setPosition(BotPositions.ARM_INTAKE);
        sArmRight.setPosition(BotPositions.ARM_INTAKE);
    }

    public void toGrab(){
        sArmLeft.setPosition(BotPositions.ARM_GRAB_PIXELS);
        sArmLeft.setPosition(BotPositions.ARM_GRAB_PIXELS);
    }

    public void toTransition(){
        sArmLeft.setPosition(BotPositions.ARM_TRANSITION_POSITION);
        sArmRight.setPosition(BotPositions.ARM_TRANSITION_POSITION);
    }

    public void toDeposit(){
        sArmLeft.setPosition(BotPositions.ARM_DEPOSIT);
        sArmRight.setPosition(BotPositions.ARM_DEPOSIT);
    }

    // original intaking position was 13cm

    public boolean inIntakeExiting() {
        if (((DistanceSensor) colorArm).getDistance(DistanceUnit.CM) <= 10)
            return true;
        else
            return false;
    }
    public boolean inIntakeEntering() {
        if (((DistanceSensor) colorArm).getDistance(DistanceUnit.CM) <= 18)
            return true;
        else
            return false;
    }

    public double getArmDistance(){
        return ((DistanceSensor) colorArm).getDistance(DistanceUnit.CM);
    }
}