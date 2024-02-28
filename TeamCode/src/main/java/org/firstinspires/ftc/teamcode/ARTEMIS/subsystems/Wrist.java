package org.firstinspires.ftc.teamcode.ARTEMIS.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
        switch (rollIndex){
            case -180:
                sGripperRoll.setPosition(BotPositions.WRIST_RIGHT_neg90_ROLL);
                break;
            case -150:
                sGripperRoll.setPosition(BotPositions.WRIST_RIGHT_neg90_ROLL);
                break;
            case -120:
                sGripperRoll.setPosition(BotPositions.WRIST_RIGHT_neg90_ROLL);
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
                sGripperRoll.setPosition(BotPositions.WRIST_LEFT_pos90_ROLL);
                break;
            case 150:
                sGripperRoll.setPosition(BotPositions.WRIST_LEFT_pos90_ROLL);
                break;
            case 180:
                sGripperRoll.setPosition(BotPositions.WRIST_LEFT_pos90_ROLL);
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

    public void rollToLeft() {
        rollIndex+=30;
//        sGripperRoll.setPosition(BotPositions.WRIST_LEFT_pos30_ROLL);
    }

    public void rollToRight() {
        rollIndex-=30;
//        sGripperRoll.setPosition(BotPositions.WRIST_RIGHT_neg30_ROLL);
    }

    public void rollToCentered() {
        rollIndex = 0;
//        sGripperRoll.setPosition(BotPositions.WRIST_ROLL_CENTERED);
    }


}
