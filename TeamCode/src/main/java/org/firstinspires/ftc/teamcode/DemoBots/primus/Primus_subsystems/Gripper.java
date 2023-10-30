package org.firstinspires.ftc.teamcode.DemoBots.primus.Primus_subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper extends SubsystemBase {
    private Servo servoL;
    private Servo servoR;

    public Gripper(HardwareMap hardwareMap) {
        servoL = hardwareMap.get(Servo.class, "sL");
        servoR = hardwareMap.get(Servo.class, "sR");
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void open(){
        servoL.setPosition(1);
        servoR.setPosition(0.2);
    }

    public void close(){
        servoL.setPosition(0.2);
        servoR.setPosition(0.6);
    }
}