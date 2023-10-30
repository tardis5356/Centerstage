package org.firstinspires.ftc.teamcode.DemoBots.optimus.Optimus_subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper extends SubsystemBase {
    private Servo servo;

    public Gripper(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "sG");
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void open(){
        servo.setPosition(0.6);
    }

    public void close(){
        servo.setPosition(0.2);
    }
}
