package org.firstinspires.ftc.teamcode.DemoBots.megatron;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public abstract class Base_Mega extends LinearOpMode{

    DcMotor mFR;
    DcMotor mFL;
    DcMotor mBR;
    DcMotor mBL;

    BNO055IMU imu;

    TouchSensor tS;

    //BNO055IMU imu;

    DcMotorSimple mLift;
   // CRServo sL;
   // CRServo sR;
    Servo sA;




    public void defineComponents(){

//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        mFR = hardwareMap.get(DcMotor.class, "mFR");
        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mBR = hardwareMap.get(DcMotor.class, "mBR");
        mBL = hardwareMap.get(DcMotor.class, "mBL");
        mLift = hardwareMap.get(DcMotorSimple.class, "mA");
        sA = hardwareMap.get(Servo.class, "sA");

        tS = hardwareMap.get(TouchSensor.class, "tS");

        imu = hardwareMap.get(BNO055IMU.class, "adafruitIMU");

        mFL.setDirection(DcMotor.Direction.REVERSE);
        mBL.setDirection(DcMotor.Direction.REVERSE);

       // RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
       // RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        // RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
    }

}
