package org.firstinspires.ftc.teamcode.ARTEMIS.subsystems;

import static org.firstinspires.ftc.teamcode.ARTEMIS.teleop.Gen1_TeleOp.highArmPosition;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist extends SubsystemBase {

    //Create Servo objects
    private Servo sGripperRoll, sGripperPitch;

    //Map Servos to physical parts
    public Wrist(HardwareMap hardwareMap) {
        sGripperRoll = hardwareMap.get(Servo.class, "sWGR"); // ROLL is side to side rotation (servo on the gripper)
        sGripperPitch = hardwareMap.get(Servo.class, "sWGP"); // TILT is up and down rotation (servo with the chain)
    }

    @Override
    public void periodic() {
    }

    public void toTransition() {
        sGripperRoll.setPosition(BotPositions.WRIST_ROLL_CENTERED);
        sGripperPitch.setPosition(BotPositions.WRIST_TILT_TRANSITION);
    }

    public void tiltToIntake() {
        sGripperPitch.setPosition(BotPositions.WRIST_TILT_INTAKE);
    }

    public void tiltToDeposit() {
        if (highArmPosition)
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
        sGripperRoll.setPosition(BotPositions.WRIST_LEFT_ROLL);
    }

    public void rollToRight() {
        sGripperRoll.setPosition(BotPositions.WRIST_RIGHT_ROLL);
    }

    public void rollToCentered() {
        sGripperRoll.setPosition(BotPositions.WRIST_ROLL_CENTERED);
    }


}
