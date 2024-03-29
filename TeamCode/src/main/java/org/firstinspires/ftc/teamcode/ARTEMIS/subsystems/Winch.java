package org.firstinspires.ftc.teamcode.ARTEMIS.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Winch extends SubsystemBase {

    //create DcMotor and Servo objects specific to this class
    //includes the winch motor, servo, and brace servos
    private DcMotor mWinch;
    private Servo sWinch;
    private ServoImplEx sBraceL;
    private ServoImplEx sBraceR;

    //hardwaremap the above objects to their physical counterparts
    public Winch(HardwareMap hardwareMap) {
        mWinch = hardwareMap.get(DcMotor.class, "mW");
//        sWinch = hardwareMap.get(Servo.class, "sW");
        sBraceL = hardwareMap.get(ServoImplEx.class, "sBL");
        sBraceR = hardwareMap.get(ServoImplEx.class, "sBR");
    }

    @Override

    // periodic is a method that runs in the background. pretty useful if you have other methods that you want to trigger automatically.
    // it is needed even if it doesn't have anything.
    public void periodic() {
    }

    //deploys the scissor lift
//    public void extendScissor() {
//        sWinch.setPosition(BotPositions.WINCH_SERVO_DEPLOYED);
//    }

    //deploys the braces
    public void extendBraces() {
        sBraceL.setPosition(BotPositions.LEFT_BRACE_EXTENDED);
        sBraceR.setPosition(BotPositions.RIGHT_BRACE_EXTENDED);
    }
    public void overextendBraces() {
        sBraceL.setPosition(BotPositions.LEFT_BRACE_OVEREXTENDED);
        sBraceR.setPosition(BotPositions.RIGHT_BRACE_OVEREXTENDED);
    }

    public void disablePWM() {
        sBraceL.setPwmDisable();
        sBraceR.setPwmDisable();
    }

    public void enablePWM() {
        sBraceL.setPwmEnable();
        sBraceR.setPwmEnable();
    }

    public void retractBraces() {
        sBraceR.setPosition(BotPositions.RIGHT_BRACE_RETRACTED);
    }

    //retracts the scissor lift
//    public void retractScissor() {
//        sWinch.setPosition(BotPositions.WINCH_SERVO_RETRACTED);
//    }

    //actually lifts the bot. pls work
    public void liftRobot() {
        mWinch.setPower(BotPositions.WINCH_MOTOR_POWER);
    }
    public void stopWinch() {
        mWinch.setPower(0);
    }
}
