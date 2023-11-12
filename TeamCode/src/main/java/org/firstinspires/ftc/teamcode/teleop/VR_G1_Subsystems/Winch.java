package org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Winch extends SubsystemBase {

    //create DcMotor and Servo objects specific to this class
    //includes the winch motor, servo, and brace servos
    private DcMotor WMotor;
    private Servo WServ;
    private Servo BraceL;
    private Servo BraceR;

    //hardwaremap the above objects to their physical counterparts
    public Winch(HardwareMap hardwareMap){
        WMotor = hardwareMap.get(DcMotor.class, "mW");
        WServ = hardwareMap.get(Servo.class, "sW");
        BraceL = hardwareMap.get(Servo.class, "sBL");
        BraceR = hardwareMap.get(Servo.class, "sBR");
    }

    @Override

    // periodic is a method that runs in the background. pretty useful if you have other methods that you want to trigger automatically.
    // it is needed even if it doesn't have anything.
    public void periodic() {}

    //deploys the scissor lift
    public void scissorDep(){
        WServ.setPosition (.2);
    }

    //deploys the braces
    public void braceDep(){
        BraceL.setPosition (.8);
        BraceR.setPosition (.8);
    }

    //retracts the scissor lift
    public void retract() {
        WServ.setPosition(.6);
    }

    //actually lifts the bot. pls work
    public void PullUp(){
        WMotor.setPower(.8);
    }
}
