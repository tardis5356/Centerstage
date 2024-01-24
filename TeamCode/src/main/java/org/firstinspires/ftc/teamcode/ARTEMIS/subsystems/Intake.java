package org.firstinspires.ftc.teamcode.ARTEMIS.subsystems;

import static org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions.INTAKE_DOWN;
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

    public void down() {
        sIntake.setPosition(INTAKE_DOWN);
    }

    public void up() {
        sIntake.setPosition(INTAKE_UP);
    }

    public void in() {
        mIntake.setPower(BotPositions.INTAKE_MOTOR_INWARD_POWER);
    }

    public void out() {
        mIntake.setPower(BotPositions.INTAKE_MOTOR_OUTWARD_POWER);
    }

    public void slowOut() {
        mIntake.setPower(BotPositions.INTAKE_MOTOR_OUTWARD_POWER_SLOW);
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

    public double getIntakePower() {
        return mIntake.getPower();
    }

    public double getIntakeCurrent() {
        return mIntake.getCurrent(CurrentUnit.MILLIAMPS);
    }
}
