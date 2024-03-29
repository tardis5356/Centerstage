package org.firstinspires.ftc.teamcode.ARTEMIS.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
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
    private ServoImplEx sBarDeploy;
    private ServoImplEx sHookDeploy;

    //hardwaremap the above objects to their physical counterparts
    public Winch(HardwareMap hardwareMap) {
        mWinch = hardwareMap.get(DcMotor.class, "mW");
//        sWinch = hardwareMap.get(Servo.class, "sW");
        sBarDeploy = hardwareMap.get(ServoImplEx.class, "sBD"); // sBD, 5 on control hub
        sHookDeploy = hardwareMap.get(ServoImplEx.class, "sHD"); // sHD, 4 on control hub
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
    public void unlatchBar() {
        sBarDeploy.setPosition(BotPositions.BAR_UNLATCHED);
    }

    public void latchBar() {
        sBarDeploy.setPosition(BotPositions.BAR_LATCHED);
    }

    public void unlatchHook() {
        sHookDeploy.setPosition(BotPositions.HOOK_UNLATCHED);
    }

    public void latchHook() {
        sHookDeploy.setPosition(BotPositions.HOOK_LATCHED);
    }

    public void disablePWM() {
        sBarDeploy.setPwmDisable();
        sHookDeploy.setPwmDisable();
    }

    public void enablePWM() {
        sBarDeploy.setPwmEnable();
        sHookDeploy.setPwmEnable();
    }

    public void liftRobot() {
        mWinch.setPower(BotPositions.WINCH_MOTOR_POWER);
    }

    public void stopWinch() {
        mWinch.setPower(0);
    }
}
