package org.firstinspires.ftc.teamcode.DemoBots.optimus.Optimus_subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;


public class Arm extends SubsystemBase {
    private DcMotor AMotor;
    private TouchSensor MSense;

    public Arm(HardwareMap hardwareMap){
        MSense = hardwareMap.get(TouchSensor.class, "armLimit");
        AMotor = hardwareMap.get(DcMotor.class, "mA");
    }

}
