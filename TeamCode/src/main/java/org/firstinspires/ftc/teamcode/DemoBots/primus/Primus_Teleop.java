package org.firstinspires.ftc.teamcode.DemoBots.primus;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp(name = "Primus_TeleOp", group="demo")
public class Primus_Teleop extends BaseClass_PP {    // LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    double zeroPosition = 0;
    boolean encoderReset = false;
    double armPosition;
    double PosDiff;
    boolean FarForward;
    boolean FarBack;
    double vArmPower;
    TouchSensor ArmLim;

    @Override
    public void runOpMode() {

        defineComponentsPrimus();
        ArmLim = hardwareMap.get(TouchSensor.class, "aTouch");
//        while(armLimit.getVoltage() > 3.0) {

//            telemetry.addData("armLimit", armLimit.getVoltage());
//            telemetry.addData("arm power", mArm.getPower());
//            telemetry.addData("arm position", mBR.getCurrentPosition());
//            telemetry.addData("zeroPosition", zeroPosition);
//            telemetry.update();
////            if (!encoderReset) {
////                if (armLimit.getVoltage() < 3.0) {
//
//                    //using mBR encoder port for mArm position
//                    mArm.setPower(0);
//                    mBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    encoderReset = true;
//                    mBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//                } else {

//                    mArm.setPower(0.5);

//                }
//            }

        //      }

        waitForStart();

        while (opModeIsActive()) {



            armPosition = mBR.getCurrentPosition() + PosDiff;

//            telemetry.addData("armLimit", armLimit.getVoltage());
            telemetry.addData("arm power", mArm.getPower());
            telemetry.addData("true arm position", mBR.getCurrentPosition());
            telemetry.addData("zeroPosition", zeroPosition);
            telemetry.addData("ArmPosition", armPosition);
            telemetry.addData("Position Diff", PosDiff);
            telemetry.addData("touchSensor", ArmLim.isPressed());
            telemetry.addData("FarForward", FarForward);
            telemetry.addData("FarBack", FarBack);
            telemetry.update();


            //Gamepad 1 Variables
            double leftY1 = gamepad1.left_stick_y;
            double rightX1 = gamepad1.right_stick_x;

            //Gamepad 2 Variables
            double leftY2 = gamepad2.left_stick_y;
            double rightTrigger2 = gamepad2.right_trigger;
            double leftTrigger2 = gamepad2.left_trigger;
            double rightY2 = gamepad2.right_stick_y;
            boolean aButton = gamepad2.a;
            boolean bButton = gamepad2.b;
            vArmPower = rightY2;

            //Drivetrain controls
            mBL.setPower(leftY1 - rightX1);
            mBR.setPower(leftY1 + rightX1);
            mFL.setPower(-leftY1 + rightX1);
            mFR.setPower(leftY1 + rightX1);

            if(ArmLim.isPressed() == false || (FarForward == false && FarBack == false)){
                mArm.setPower(vArmPower);
//                FarForward = false;
            }
            if(ArmLim.isPressed() == true){
                armPosition = 1000;
                PosDiff = armPosition - mBR.getCurrentPosition();
                FarForward = true;
                if(rightY2 < 0){
                    vArmPower = (0);
                }

                mArm.setPower(vArmPower);
            }
            if(armPosition < 2000 && FarForward == true){
                mArm.setPower(vArmPower);
            }
            if(armPosition >= 2000){
                FarForward = false;
            }
            if(armPosition >= 5000 || FarBack == true){
                FarBack = true;
                mArm.setPower(-1);
            }
            if(armPosition >= 4000){
                FarBack = false;
            }

            //Arm Motor Controls

            //make sure blue wire of the sensor is closest to the 0.1 side (to the outside of the
            //if statement makes sure that arm can only drive up if limit switch is pressed
            //if limit switch is pressed, voltage is around 0, if not pressed, voltage is around 3.3
//            if(armLimit.getVoltage() < 3.0){
//
//                if(leftY2 > 0) {
//                    mArm.setPower(0);
//                }
//
//                else{
//                    mArm.setPower(leftY2);
//                }
//            }else if(mBR.getCurrentPosition() < -9000){
//
//                if(leftY2 < 0) {
//                    mArm.setPower(0);
//                }
//
//                else{
//                    mArm.setPower(leftY2);
//                }
//            } else{
//
//                mArm.setPower(leftY2);
//
//            }
//            mArm.setPower(leftY2);
//            if (aButton) {
//                if (mBR.getCurrentPosition() > 1020) {
//                    mArm.setPower(-1);
//                } else if (mBR.getCurrentPosition() < 980) {
//                    mArm.setPower(1);
//                } else if (mBR.getCurrentPosition() > 980 && mBR.getCurrentPosition() < 1020) {
//                    mArm.setPower(0);
//                }
//            }
//            if (bButton) {
//                mBR.setTargetPosition(1000);
//            }

            if (rightTrigger2 != 0) {
                //sL.setPosition(0.3);
                sR.setPosition(0.6);
            } else {
                sR.setPosition(0.25); //0.3
            }
            if (leftTrigger2 != 0) {
                sL.setPosition(0.35);
                //sR.setPosition(0.3);
            } else {
                sL.setPosition(0.75); //0.3
            }


        }


    }


}