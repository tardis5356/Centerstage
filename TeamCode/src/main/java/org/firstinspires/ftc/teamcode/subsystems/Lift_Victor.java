package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
@Disabled
public class Lift_Victor extends SubsystemBase {
//    private PIDController controller,
//            extendController, retractController;



    private DcMotorEx LLiftM, RLiftM;
    private TouchSensor LSence;

    public Lift_Victor(HardwareMap hardwareMap){
        LLiftM = hardwareMap.get(DcMotorEx.class, "mLL");
        RLiftM = hardwareMap.get(DcMotorEx.class, "mLR");
        LSence = hardwareMap.get(TouchSensor.class, "lT");
    }

    @Override
    public void periodic() {}

//    public void Raise(){
//        LLiftM.setPower(-.2);
//        RLiftM.setPower(-.2);
//    }
    public void Lower(){
        if(LSence.isPressed() == false){
            LLiftM.setPower(.2);
            RLiftM.setPower(.2);
        }
        else{
            LLiftM.setPower(0);
            RLiftM.setPower(0);
        }
    }
//    public void Stay(){
//        LLiftM.setPower(0);
//        RLiftM.setPower(0);
//    }
}
