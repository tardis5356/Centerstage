package org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Lift extends SubsystemBase {
    private DcMotor LLiftM;
    private DcMotor RLiftM;
    private TouchSensor LSence;
    public Lift(HardwareMap hardwareMap){
        LLiftM = hardwareMap.get(DcMotor.class, "mLL");
        RLiftM = hardwareMap.get(DcMotor.class, "mLR");
        LSence = hardwareMap.get(TouchSensor.class, "lT");
    }
}
