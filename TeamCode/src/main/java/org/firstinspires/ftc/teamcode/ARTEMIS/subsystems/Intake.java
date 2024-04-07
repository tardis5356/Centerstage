package org.firstinspires.ftc.teamcode.ARTEMIS.subsystems;

import static org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions.INTAKE_DOWN_FIFTH_PIXEL;
import static org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions.INTAKE_DOWN_FIRST_PIXEL;
import static org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions.INTAKE_DOWN_FOURTH_PIXEL;
import static org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions.INTAKE_DOWN_SECOND_PIXEL;
import static org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions.INTAKE_DOWN_TELEOP;
import static org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions.INTAKE_DOWN_THIRD_PIXEL;
import static org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions.INTAKE_UP;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake extends SubsystemBase {
    private DcMotorEx mIntake;
    private Servo sIntake;
    private ColorSensor colorLeft, colorRight;

    public Intake(HardwareMap hardwareMap) {
        mIntake = hardwareMap.get(DcMotorEx.class, "mI");
        sIntake = hardwareMap.get(Servo.class, "sI");

        colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        colorRight = hardwareMap.get(ColorSensor.class, "colorRight");
    }

    @Override
    public void periodic() {
    }

    public void downTeleOp() {
        sIntake.setPosition(INTAKE_DOWN_TELEOP);
    }

    public void downFirstPixel() {
        sIntake.setPosition(INTAKE_DOWN_FIRST_PIXEL);
    }

    public void downSecondPixel() { sIntake.setPosition(INTAKE_DOWN_SECOND_PIXEL); }

    public void downThirdPixel() {
        sIntake.setPosition(INTAKE_DOWN_THIRD_PIXEL);
    }

    public void downFourthPixel() {
        sIntake.setPosition(INTAKE_DOWN_FOURTH_PIXEL);
    }

    public void downFifthPixel() {
        sIntake.setPosition(INTAKE_DOWN_FIFTH_PIXEL);
    }

    public void up() {
        sIntake.setPosition(INTAKE_UP);
    }

    public void in() {
        mIntake.setPower(BotPositions.INTAKE_MOTOR_INWARD_POWER);
    }
    public void inSlow() {
        mIntake.setPower(BotPositions.INTAKE_MOTOR_INWARD_POWER_SLOW);
    }

    public void out() {
        mIntake.setPower(BotPositions.INTAKE_MOTOR_OUTWARD_POWER);
    }

    public void slowOut() {
        mIntake.setPower(BotPositions.INTAKE_MOTOR_OUTWARD_POWER_SLOW);
    }

    public void superSlowOut() {
        mIntake.setPower(BotPositions.INTAKE_MOTOR_OUTWARD_POWER_SUPER_SLOW);
    }

    public void stop() {
        mIntake.setPower(0);
    }

    public double getIntakeLeftDistance() {
        return ((DistanceSensor) colorLeft).getDistance(DistanceUnit.CM);
    }

    public double getIntakeRightDistance() {
        return ((DistanceSensor) colorRight).getDistance(DistanceUnit.CM);
    }

    public void disableLEDs() {
        colorLeft.enableLed(false);
        colorRight.enableLed(false);
    }


    public double getIntakePower() {
        return mIntake.getPower();
    }

    public double getIntakeCurrent() {
        return mIntake.getCurrent(CurrentUnit.MILLIAMPS);
    }
}
