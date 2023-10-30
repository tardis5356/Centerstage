package org.firstinspires.ftc.teamcode.DemoBots.optimus.Optimus_subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist extends SubsystemBase {
    private Servo servo;

    public Wrist(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "sG");
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void intake(){
        servo.setPosition(0.6);
    }

    public void deliver(){
        servo.setPosition(0.2);
    }
}
