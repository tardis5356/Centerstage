package org.firstinspires.ftc.teamcode.ARTEMIS.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class DroneLauncher extends SubsystemBase{
    private Servo sDroneLauncher;

    public DroneLauncher(HardwareMap hardwareMap){
        sDroneLauncher = hardwareMap.get(Servo.class, "sDL");
    }

    @Override
    public void periodic(){}

    public void latch(){
        sDroneLauncher.setPosition(BotPositions.DRONE_LATCHED);
    }

    public void unlatch(){
        sDroneLauncher.setPosition(BotPositions.DRONE_UNLATCHED);
    }
}
