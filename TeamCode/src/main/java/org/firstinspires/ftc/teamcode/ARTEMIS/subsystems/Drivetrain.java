package org.firstinspires.ftc.teamcode.ARTEMIS.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drivetrain extends SubsystemBase {
    private DcMotor mFL;
    private DcMotor mFR;
    private DcMotor mBL;
    private DcMotor mBR;

//    BNO055IMU imu;
    private IMU imu;

    double startingErrorRads = 0;
    double startingOffsetRads = 0;

//    double correctedAngleDegrees = 0;
    
    public Drivetrain(HardwareMap hardwareMap) {
        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mFR = hardwareMap.get(DcMotor.class, "mFR");
        mBL = hardwareMap.get(DcMotor.class, "mBL");
        mBR = hardwareMap.get(DcMotor.class, "mBR");

//        imu = hardwareMap.get(BNO055IMU.class, "imuEx");

//        imu.initialize(new BNO055IMU.Parameters());


        imu = hardwareMap.get(IMU.class, "imuEx");

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                        )
                )
        );

        imu.resetYaw();
    }



    public void setStartingOffsetDegs(int offsetDeg){
        startingOffsetRads = Math.toRadians(offsetDeg);
    }

    public double getStartingOffsetDegs(){
        return Math.toDegrees(startingOffsetRads);
    }

    public double getRawYawDegrees(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    @Override
    public void periodic() {

    }

    public double getYawDegrees(){
        return getRawYawDegrees() + getStartingOffsetDegs();
    }

    public double getYawRadians(){
        return Math.toRadians(getYawDegrees());
    }

    public double getPose2dYawRads(){
        return Math.toRadians((getYawDegrees() + 360) % 360);
    }

    public double getPose2dYawDegs(){
        return ((getYawDegrees() + 360) % 360);
    }


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


    /*
    public BNO055IMU getImu() {
        return imu;
    }

    public Orientation getIMUOrientation(){
        return imu.getAngularOrientation();
    }

    public void setStartingError(){
        startingErrorRads = AngleUnit.normalizeDegrees(-imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle);
    }
    public void setStartingOffsetDegs(int offsetDeg){
        startingOffsetRads = Math.toRadians(offsetDeg);
    }

    public double getYawRadians(){
        return AngleUnit.normalizeRadians(-imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle-startingErrorRads+startingOffsetRads);
    }

    public double getYawDegrees(){
        return AngleUnit.normalizeDegrees(-imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle-Math.toDegrees(startingErrorRads)+Math.toDegrees(startingOffsetRads));
    }

    public double getRawYawDegrees(){
        return AngleUnit.normalizeDegrees(-imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
    }*/

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