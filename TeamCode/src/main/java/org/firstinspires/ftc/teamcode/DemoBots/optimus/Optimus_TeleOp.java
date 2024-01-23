package org.firstinspires.ftc.teamcode.DemoBots.optimus;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

//@Disabled
@TeleOp(name="Optimus_TeleOp")
//@Disabled
public class Optimus_TeleOp extends LinearOpMode{

    // create objects and give them classes
    DcMotor mL;
    DcMotor mR;
    DcMotor mA;

    Servo sG;
    Servo sW;
    TouchSensor limit;
    double wristPosition = .7;
    double ArmPosition;
    double PositionDiff;
    boolean FarBack;
// initialization

    @Override
    public void runOpMode (){
// map objects to motors/servos
        mL = hardwareMap.get(DcMotor.class, "mL");
        mR = hardwareMap.get(DcMotor.class, "mR");
        mA = hardwareMap.get(DcMotor.class, "mA");
        sG = hardwareMap.get(Servo.class, "sG");
        sW = hardwareMap.get(Servo.class, "sW");
        limit = hardwareMap.get(TouchSensor.class, "armLimit");



// program has startedz
        waitForStart();

        while(opModeIsActive())   {
            //chasis

            ArmPosition = mA.getCurrentPosition() + PositionDiff;

            double LeftstickY = gamepad1.left_stick_y;
            double RightstickX = gamepad1.right_stick_x;
            mL.setPower((LeftstickY/1.5)-(RightstickX/2));
            mR.setPower((-LeftstickY/1.5)-(RightstickX/2));

            //gripper
            double RightTrigger = gamepad2.right_trigger;

            if (RightTrigger != 0){
                sG.setPosition(0.35);
            } else {
              sG.setPosition(.6);
            }

            //wrist
            boolean rB2 = gamepad2.dpad_down;
            boolean lB2 = gamepad2.dpad_up;
            boolean dpr2 = gamepad2.dpad_right;

            if (lB2) {
                wristPosition = .7;
            }
//            else if (lB2){
//                wristPosition = 0;
//
//            }
            else if (rB2){
                wristPosition = .35;
            }
            else if (rB2 == false && lB2 == false){

            }

            sW.setPosition(wristPosition);

            //arm

            double RightstickY2 = gamepad2.right_stick_y;

            if(limit.isPressed() == false && ArmPosition >= 0){
                mA.setPower((-RightstickY2)/2);
                FarBack = false;
            }
            else if(ArmPosition < -100 && limit.isPressed() == false){
                FarBack = true;
                mA.setPower(.5);
            }
            else if (ArmPosition >= -100 && ArmPosition < 0 && FarBack == true){
                mA.setPower(.5);
            }
            else if (ArmPosition >= -100 && ArmPosition < 0 && FarBack == false){
                mA.setPower((-RightstickY2)/2);
            }
//            else if(FarBack = true){
//                mA.setPower(0.5);
//            }
//            else if(ArmPosition >= 0){
//                FarBack = false;
//            }
            else if (limit.isPressed() == true){
                mA.setPower(-0.5);
                ArmPosition = 3750;
                PositionDiff = ArmPosition - mA.getCurrentPosition();
            }


            telemetry.addData("trigger", gamepad2.right_trigger);
            telemetry.addData("lbump", gamepad2.left_bumper);
            telemetry.addData("rbump", gamepad2.right_bumper);
            telemetry.addData("WristPosition", wristPosition);
            telemetry.addData("limit", limit.isPressed());
            telemetry.addData("ArmPosition", ArmPosition);
            telemetry.addData("TrueArmPosition", mA.getCurrentPosition());
            telemetry.update();


        }



    }
}
