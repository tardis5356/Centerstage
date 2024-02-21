package org.firstinspires.ftc.teamcode.ARTEMIS.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Drivetrain extends SubsystemBase {
    private DcMotor mFL;
    private DcMotor mFR;
    private DcMotor mBL;
    private DcMotor mBR;

    BNO055IMU imu;

    public Drivetrain(HardwareMap hardwareMap) {
        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mFR = hardwareMap.get(DcMotor.class, "mFR");
        mBL = hardwareMap.get(DcMotor.class, "mBL");
        mBR = hardwareMap.get(DcMotor.class, "mBR");

        imu = hardwareMap.get(BNO055IMU.class, "imuEx");

        imu.initialize(new BNO055IMU.Parameters());
    }

    @Override
    public void periodic() {}

    public void driveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        mFL.setPower(leftFrontPower);
        mFR.setPower(rightFrontPower);
        mBL.setPower(leftBackPower);
        mBR.setPower(rightBackPower);
    }

    public BNO055IMU getImu() {
        return imu;
    }

    public Orientation getIMUOrientation(){
        return imu.getAngularOrientation();
    }

    public double getYawRadians(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle+Math.toRadians(270);
    }

    public double getYawDegrees(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle+(270);
    }

    // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
    // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
    // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
    public void setCamera(String webcam) {
        if (webcam.toLowerCase() == "front") {
            mFL.setDirection(DcMotor.Direction.REVERSE);
            mBL.setDirection(DcMotor.Direction.REVERSE);
            mFR.setDirection(DcMotor.Direction.REVERSE);
            mBR.setDirection(DcMotor.Direction.FORWARD);
        } else {
            // back webcam
            mFL.setDirection(DcMotor.Direction.FORWARD);
            mBL.setDirection(DcMotor.Direction.FORWARD);
            mFR.setDirection(DcMotor.Direction.FORWARD);
            mBR.setDirection(DcMotor.Direction.REVERSE);
        }
    }
}