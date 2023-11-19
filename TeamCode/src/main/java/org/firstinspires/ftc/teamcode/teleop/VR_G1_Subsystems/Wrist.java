package org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist extends SubsystemBase {

    //Create Servo objects
    private Servo ServRotate, ServTilt;

    //Map Servos to physical parts
    public Wrist (HardwareMap hardwareMap){
        ServRotate = hardwareMap.get(Servo.class, "sWGR");
        ServTilt = hardwareMap.get(Servo.class,"sWGP");
    }

    @Override
    public void periodic(){}

    public void WristToIntakePrep(){
        ServRotate.setPosition(BotPositions.SQUARE_WRIST_ROLL_POSITION);
        ServTilt.setPosition(BotPositions.WRIST_INTAKE_PREP_TILT);
    }

    public void TiltToIntake(){
        ServTilt.setPosition(BotPositions.WRIST_INTAKE_TILT);
    }

    public void TiltToOutput(){
        ServTilt.setPosition(BotPositions.WRIST_OUTPUT_TILT);
    }

    public void rotateLeft(){
        ServRotate.setPosition(BotPositions.LEFT_WRIST_ROLL_POSITION);
    }

    public void rotateRight(){
        ServRotate.setPosition(BotPositions.RIGHT_WRIST_ROLL_POSITION);
    }

    public void rotateSquare(){
        ServRotate.setPosition(BotPositions.SQUARE_WRIST_ROLL_POSITION);
    }


}
