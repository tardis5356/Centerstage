package org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Winch extends SubsystemBase {
    private DcMotor WMotor;
    public Winch(HardwareMap hardwareMap){
        WMotor = hardwareMap.get(DcMotor.class, "mW");
    }
//    @Override
//    public void periodic() {}
//
//    public void Raise(){
//
//    }
//    public void Lower(){
//
//    }
}
