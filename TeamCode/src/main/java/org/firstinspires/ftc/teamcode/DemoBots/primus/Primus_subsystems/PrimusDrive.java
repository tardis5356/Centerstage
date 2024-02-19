package org.firstinspires.ftc.teamcode.DemoBots.primus.Primus_subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class PrimusDrive extends SubsystemBase {
    private DcMotor mFR, mFL, mBR, mBL;

    BNO055IMU imu;

    Orientation orientation;

    public PrimusDrive(HardwareMap hardwareMap) {
        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mFR = hardwareMap.get(DcMotor.class, "mFR");
        mBL = hardwareMap.get(DcMotor.class, "mBL");
        mBR = hardwareMap.get(DcMotor.class, "mBR");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(new BNO055IMU.Parameters());
    }

    @Override

    // periodic is a method that runs in the background. pretty useful if you have other methods that you want to trigger automatically.
    // it is needed even if it doesn't have anything.
    public void periodic() {
    }

    public void DriveForward() {
        mFL.setPower(-.75);
        mBL.setPower(-.75);
        mFR.setPower(.75);
        mBR.setPower(.75);
    }

    public void DriveBackward() {
        mFL.setPower(.75);
        mBL.setPower(.75);
        mFR.setPower(-.75);
        mBR.setPower(-.75);
    }

    public void TurnLeft() {
        mFR.setPower(.5);
        mFL.setPower(.5);
        mBR.setPower(.5);
        mBL.setPower(.5);
    }

    public void TurnRight() {
        mFR.setPower(-.5);
        mFL.setPower(-.5);
        mBR.setPower(-.5);
        mBL.setPower(-.5);
    }

    public void slowTurnLeft() {
        mFR.setPower(.05);
        mFL.setPower(.05);
        mBR.setPower(.05);
        mBL.setPower(.05);
    }

    public void slowTurnRight() {
        mFR.setPower(-.05);
        mFL.setPower(-.05);
        mBR.setPower(-.05);
        mBL.setPower(-.05);
    }

    public void stopDriving() {
        mFR.setPower(0);
        mFL.setPower(0);
        mBR.setPower(0);
        mBL.setPower(0);
    }

    public float getIMUAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void resetIMUAngle() {

    }

    public BNO055IMU getImu() {
        return imu;
    }
}
