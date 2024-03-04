package org.firstinspires.ftc.teamcode.ARTEMIS.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;

public class Wrist extends SubsystemBase {

    //Create Servo objects
    private Servo sGripperRoll, sGripperPitch;

    private int rollIndex = 0;

    //Map Servos to physical parts
    public Wrist(HardwareMap hardwareMap) {
        sGripperRoll = hardwareMap.get(Servo.class, "sWGR"); // ROLL is side to side rotation (servo on the gripper)
        sGripperPitch = hardwareMap.get(Servo.class, "sWGP"); // TILT is up and down rotation (servo with the chain)
    }

    @Override
    public void periodic() {
        if(rollIndex > 180)
            rollIndex = -180;
        else if(rollIndex < -180)
            rollIndex = 180;

        switch (rollIndex){
            case -180:
                sGripperRoll.setPosition(BotPositions.WRIST_RIGHT_neg180_ROLL);
                break;
            case -150:
                sGripperRoll.setPosition(BotPositions.WRIST_RIGHT_neg150_ROLL);
                break;
            case -120:
                sGripperRoll.setPosition(BotPositions.WRIST_RIGHT_neg120_ROLL);
                break;
            case -90:
                sGripperRoll.setPosition(BotPositions.WRIST_RIGHT_neg90_ROLL);
                break;
            case -60:
                sGripperRoll.setPosition(BotPositions.WRIST_RIGHT_neg60_ROLL);
                break;
            case -30:
                sGripperRoll.setPosition(BotPositions.WRIST_RIGHT_neg30_ROLL);
                break;
            case 0:
            default:
                sGripperRoll.setPosition(BotPositions.WRIST_ROLL_CENTERED);
                break;
            case 30:
                sGripperRoll.setPosition(BotPositions.WRIST_LEFT_pos30_ROLL);
                break;
            case 60:
                sGripperRoll.setPosition(BotPositions.WRIST_LEFT_pos60_ROLL);
                break;
            case 90:
                sGripperRoll.setPosition(BotPositions.WRIST_LEFT_pos90_ROLL);
                break;
            case 120:
                sGripperRoll.setPosition(BotPositions.WRIST_LEFT_pos120_ROLL);
                break;
            case 150:
                sGripperRoll.setPosition(BotPositions.WRIST_LEFT_pos150_ROLL);
                break;
            case 180:
                sGripperRoll.setPosition(BotPositions.WRIST_LEFT_pos180_ROLL);
                break;
        }
    }

    public void setRollIndex(int index){ rollIndex = index; }

    public void toTransition() {
//        sGripperRoll.setPosition(BotPositions.WRIST_ROLL_CENTERED);
        rollIndex = 0;
        sGripperPitch.setPosition(BotPositions.WRIST_TILT_TRANSITION);
    }

    public void tiltToIntake() {
        sGripperPitch.setPosition(BotPositions.WRIST_TILT_INTAKE);
    }

    public void tiltToDeposit() {
        if (BotPositions.ARM_HIGH_POSITION)
            sGripperPitch.setPosition(BotPositions.WRIST_TILT_DEPOSIT);
        else
            sGripperPitch.setPosition(BotPositions.WRIST_TILT_DEPOSIT_LOW);
    }

    public void tiltToDepositHigh() {
        sGripperPitch.setPosition(BotPositions.WRIST_TILT_DEPOSIT);
    }

    public void tiltToDropPurplePixel() {
        sGripperPitch.setPosition(BotPositions.WRIST_TILT_DROP_PURPLE);
    }

    public void rollToLeft30() {
        rollIndex+=30;
//        sGripperRoll.setPosition(BotPositions.WRIST_LEFT_pos30_ROLL);
    }

    public void rollToRight30() {
        rollIndex-=30;
//        sGripperRoll.setPosition(BotPositions.WRIST_RIGHT_neg30_ROLL);
    }

    public void rollToLeft60() {
        rollIndex+=60;
//        sGripperRoll.setPosition(BotPositions.WRIST_LEFT_pos30_ROLL);
    }

    public void rollToRight60() {
        rollIndex-=60;
//        sGripperRoll.setPosition(BotPositions.WRIST_RIGHT_neg30_ROLL);
    }

    public void rollToCentered() {
        rollIndex = 0;
//        sGripperRoll.setPosition(BotPositions.WRIST_ROLL_CENTERED);
    }

    public void rollToPurpleAuto(){
        if(!SampleMecanumDrive.flipPose) //red
            rollIndex = 60; //90
        else
            rollIndex = -60;
    }

//    public void rollToYellowAuto(){
//        if(!SampleMecanumDrive.flipPose) //red
//            rollIndex = 60; //90
//        else
//            rollIndex = -90;
//    }

}
