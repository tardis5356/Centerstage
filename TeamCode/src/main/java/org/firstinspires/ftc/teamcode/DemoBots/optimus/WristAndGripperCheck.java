package org.firstinspires.ftc.teamcode.DemoBots.optimus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Wrist_and_Gripper_Check", group="demo")
public class WristAndGripperCheck extends LinearOpMode {
    Servo sG;
    Servo sW;
    @Override
        public void runOpMode (){
        sG = hardwareMap.get(Servo.class, "sG");
        sW = hardwareMap.get(Servo.class, "sW");

     waitForStart();
            while(opModeIsActive())   {
                sG.setPosition(1);
                sW.setPosition(1);
            }
        
    }


}

