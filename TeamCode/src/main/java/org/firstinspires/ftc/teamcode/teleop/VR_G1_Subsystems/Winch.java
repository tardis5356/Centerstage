package org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Winch extends SubsystemBase {
    private DcMotor WMotor;
    private Servo LinServ;
    private Servo BraceL;
    private Servo BraceR;

    public Winch(HardwareMap hardwareMap){
        WMotor = hardwareMap.get(DcMotor.class, "mW");
        LinServ = hardwareMap.get(Servo.class, "sW");
        BraceL = hardwareMap.get(Servo.class, "sBL");
        BraceR = hardwareMap.get(Servo.class, "sBR");
    }

    @Override
    public void periodic() {}

    public void scissorDep(){
        LinServ.setPosition (.2);
    }
    public void braceDep(){
        BraceL.setPosition (.5);
        BraceR.setPosition (.5);
    }
    public void retract() {
        LinServ.setPosition(.7);
    }
    public void PullUp(){
        WMotor.setPower(.8);
    }
}
