package org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Lift extends SubsystemBase {
//    private PIDController controller,
//            extendController, retractController;



    private DcMotorEx LLiftM, RLiftM;
    private TouchSensor LSence;
    public Lift(HardwareMap hardwareMap){
        LLiftM = hardwareMap.get(DcMotorEx.class, "mLL");
        RLiftM = hardwareMap.get(DcMotorEx.class, "mLR");
        LSence = hardwareMap.get(TouchSensor.class, "lT");
    }

    @Override
    public void periodic() {}

    public void Raise(){
        LLiftM.setPower(-.2);
        RLiftM.setPower(-.2);
    }
    public void Lower(){
        LLiftM.setPower(.2);
        RLiftM.setPower(.2);
    }
    public void Stay(){
        LLiftM.setPower(0);
        RLiftM.setPower(0);
    }
}
