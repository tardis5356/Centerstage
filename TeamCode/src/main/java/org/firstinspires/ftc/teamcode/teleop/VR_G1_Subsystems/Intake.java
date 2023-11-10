package org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.LEDs;

public class Intake extends SubsystemBase {
    private DcMotorEx IntakeM;

    public Intake(HardwareMap hardwareMap){
        IntakeM = hardwareMap.get(DcMotorEx.class, "mI");
    }

    @Override
    public void periodic(){}

    public void in() {
        IntakeM.setPower(-0.8);
//        LEDstate = "Intaking";
    }

    public void out(){
        IntakeM.setPower(.3);
    }

    public void stop() {
        IntakeM.setPower(0);
    }

}
