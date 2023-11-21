package org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {
    private DcMotorEx mIntake;

    public Intake(HardwareMap hardwareMap){
        mIntake = hardwareMap.get(DcMotorEx.class, "mI");
    }

    @Override
    public void periodic(){}

    public void in() {
        mIntake.setPower(BotPositions.INTAKE_MOTOR_INWARD_POWER);
//        LEDstate = "Intaking";
    }

    public void out(){
        mIntake.setPower(BotPositions.INTAKE_MOTOR_OUTWARD_POWER);
    }

    public void stop() {
        mIntake.setPower(0);
    }

}
