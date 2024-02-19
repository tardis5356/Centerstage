package org.firstinspires.ftc.teamcode.DemoBots.optimus.Optimus_subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions;

public class OptimusDrive extends SubsystemBase {
    private DcMotor LeftDrive;
    private DcMotor RightDrive;

    BNO055IMU imu;

    Orientation orientation;

    public OptimusDrive(HardwareMap hardwareMap) {
        LeftDrive = hardwareMap.get(DcMotor.class, "mL");
        RightDrive = hardwareMap.get(DcMotor.class, "mR");

        LeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(new BNO055IMU.Parameters());
    }

    @Override

    // periodic is a method that runs in the background. pretty useful if you have other methods that you want to trigger automatically.
    // it is needed even if it doesn't have anything.
    public void periodic() {
//        orientation = imu.getAngularOrientation();
    }

    public void DriveForward() {
        LeftDrive.setPower(-.75);
        RightDrive.setPower(.75);
    }

    public void DriveBackward() {
        LeftDrive.setPower(.75);
        RightDrive.setPower(-.75);
    }

    public void TurnLeft() {
        LeftDrive.setPower(.5);
        RightDrive.setPower(.5);
    }

    public void TurnRight() {
        LeftDrive.setPower(-0.5);
        RightDrive.setPower(-0.5);
    }

    public void slowTurnLeft() {
        LeftDrive.setPower(.25);
        RightDrive.setPower(.25);
    }

    public void slowTurnRight() {
        LeftDrive.setPower(-0.05);
        RightDrive.setPower(-0.05);
    }

    public void adjustLeft() {
        LeftDrive.setPower(-.75);
        RightDrive.setPower(.78);
    }

    public void adjustRight() {
        LeftDrive.setPower(-.78);
        RightDrive.setPower(.75);
    }

    public void stopDriving() {
        LeftDrive.setPower(0);
        RightDrive.setPower(0);
    }

    public double getmLPower(){
        return LeftDrive.getPower();
    }

    public float getIMUAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void resetIMUAngle(){

    }

    public BNO055IMU getImu() {
        return imu;
    }
}
