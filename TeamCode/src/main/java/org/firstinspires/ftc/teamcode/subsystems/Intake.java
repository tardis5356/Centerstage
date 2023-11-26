package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake extends SubsystemBase {
    private DcMotorEx mIntake;
    private ColorSensor colorLeft, colorRight;

    public Intake(HardwareMap hardwareMap) {
        mIntake = hardwareMap.get(DcMotorEx.class, "mI");

        colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        colorRight = hardwareMap.get(ColorSensor.class, "colorRight");
    }

    @Override
    public void periodic() {
    }

    public void in() {
        mIntake.setPower(BotPositions.INTAKE_MOTOR_INWARD_POWER);
//        LEDstate = "Intaking";
    }

    public void out() {
        mIntake.setPower(BotPositions.INTAKE_MOTOR_OUTWARD_POWER);
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

}
