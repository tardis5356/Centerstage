package org.firstinspires.ftc.teamcode.DemoBots.megatron;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.DemoBots.megatron.Base_Mega;
//@Disabled
@TeleOp(name="Mega_FieldC(Outreach)")


public class Mega_Gyro extends Base_Mega {

    Servo sL;
//    CRServo sR;

    double AngleOffset = 0;

    @Override
    public void runOpMode(){



        defineComponents();

        sL = hardwareMap.get(Servo.class, "sL");
//        sR = hardwareMap.get(CRServo.class, "sR");

        Mega_drivetrain mD = new Mega_drivetrain(mFL, mFR, mBL, mBR);

        imu.initialize(new BNO055IMU.Parameters());

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "adafruitIMU");
        imu.initialize(parameters);
//          Mega_Gripper mG = new Mega_Gripper(sL, sR);

        Mega_Lift mL = new Mega_Lift(mLift, mBL);

        Mega_Arm mA = new Mega_Arm(sA);

        waitForStart();

        while(opModeIsActive()){

           Orientation orientation = imu.getAngularOrientation();

            double gP1sLy = -gamepad1.left_stick_y;
            double gP1sLx = gamepad1.left_stick_x;
            double gP1sRx = gamepad1.right_stick_x;

            boolean gP1dPU = gamepad2.dpad_up;
            boolean gP1dPD = gamepad2.dpad_down;
            boolean gP2a = gamepad2.a;
            boolean gP1a = gamepad1.a;
            boolean gP1Bl = gamepad2.left_bumper;
            boolean gP1Br = gamepad2.right_bumper;
            boolean gP1dPL = gamepad2.dpad_left;
            boolean gP1dPR = gamepad2.dpad_right;

            double CurrentAngle = orientation.firstAngle;

            if (gP1a == true){

               AngleOffset = CurrentAngle;

            }

            double CalcAngle = CurrentAngle - AngleOffset;

            double XVector = (gP1sLx*Math.cos(Math.toRadians(CalcAngle)))+(gP1sLy*Math.sin(Math.toRadians(CalcAngle)));
            double YVector = -(gP1sLx*Math.sin(Math.toRadians(CalcAngle)))+(gP1sLy*Math.cos(Math.toRadians(CalcAngle)));

           mD.Mega_drive(YVector/1.5, XVector/1.5, gP1sRx/1.5);

            if (tS.isPressed() == false) {
            mL.rise(gP1dPU, gP1dPD);
            mL.setToZero(gP2a);
            }
            else if (tS.isPressed() == true){
                mL.rise(true,false);
                mL.setToZero(true);
            }

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

//            telemetry.addData("sL power", sL.getPower());
//            telemetry.addData("sR power", sR.getPower());
            telemetry.addData("lift Position", mL.getLiftPosition());
            telemetry.addData("touch ye?", tS.isPressed());


            telemetry.addData("Gyro_Position",angles.firstAngle);

            // YawPitchRollAngles orientation = imu.getAngularOrientation();
            // AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);





//            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.thirdAngle);
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.firstAngle);
        //          telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.secondAngle);

           // telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
           // telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
           // telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);

            telemetry.addData("Calculated Angle", CalcAngle);

            telemetry.update();

//            if (gP1Bl == true){
//                mG.Mega_Grip(1,-1);gbv
//            }
//            else if(gP1Br == true){
//                mG.Mega_Grip(-1,1);
//            }
//            else{
//               // mG.Mega_Grip(0,0);
//            }

            if (gP1dPR && mL.getLiftPosition() <  -100){
                mA.armSet(0.6);
            }
            else if(gP1dPL && mL.getLiftPosition() <  -100){
                mA.armSet(0.22);
            }
            else{

            }



            if(gP1Bl || gP1Br){
                sL.setPosition(.3);
//                sR.setPower(-1);
            }
            else{
                sL.setPosition(.5);
//                sR.setPower(1);
            }
//            else {
//                sL.setPower(0);
//                sR.setPower(0);
//
//            }



        }


    }
}
