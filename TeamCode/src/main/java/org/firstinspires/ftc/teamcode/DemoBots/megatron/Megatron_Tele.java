package org.firstinspires.ftc.teamcode.DemoBots.megatron;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DemoBots.megatron.Base_Mega;
@Disabled
@TeleOp(name="Mega_BotC", group="demo")
public class Megatron_Tele extends Base_Mega {

    Servo sL;
//    Servo sR;



    @Override
    public void runOpMode(){

        defineComponents();

        sL = hardwareMap.get(Servo.class, "sL");
//        sR = hardwareMap.get(Servo.class, "sR");

        Mega_drivetrain mD = new Mega_drivetrain(mFL, mFR, mBL, mBR);

//        Mega_Gripper mG = new Mega_Gripper(sL, sR);

        Mega_Lift mL = new Mega_Lift(mLift, mBL);

        Mega_Arm mA = new Mega_Arm(sA);

        waitForStart();

        while(opModeIsActive()){

            double gP1sLy = -gamepad1.left_stick_y;
            double gP1sLx = gamepad1.left_stick_x;
            double gP1sRx = gamepad1.right_stick_x;
            boolean gP2dPU = gamepad1.dpad_up;
            boolean gP2dPD = gamepad1.dpad_down;

            boolean gP1dPU = gamepad2.dpad_up;
            boolean gP1dPD = gamepad2.dpad_down;
            boolean gP1a = gamepad2.a;
            boolean gP1Bl = gamepad2.left_bumper;
            boolean gP1Br = gamepad2.right_bumper;
            boolean gP1dPL = gamepad2.dpad_left;
            boolean gP1dPR = gamepad2.dpad_right;


            mD.Mega_drive(gP1sLy/1.5, gP1sLx/1.5, gP1sRx/1.5);

               if (tS.isPressed() == false) {
                   mL.rise(gP1dPU, gP1dPD);
                   mL.setToZero(gP1a);
               }
               else if (tS.isPressed() == true){
            mL.rise(true,false);
            mL.setToZero(true);
        }

//            telemetry.addData("sL power", sL.getPower());
//            telemetry.addData("sR power", sR.getPower());
            telemetry.addData("lift Position", mL.getLiftPosition());
            telemetry.addData("touch ye?", tS.isPressed());
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

            if (gP1dPR == true && mL.getLiftPosition() <  -100){
                mA.armSet(0.6);
            }
            else if(gP1dPL == true && mL.getLiftPosition() <  -100){
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
//               sR.setPower(1);
            }
//            else {
//                sL.setPower(0);
//                sR.setPower(0);
//          }



        }


    }
}
