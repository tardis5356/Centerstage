package org.firstinspires.ftc.teamcode.DemoBots.optimus;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Disabled
@TeleOp(name="Optimus_Centerstage_TeleOp", group="demo")
public class Optimus_CenterStage extends LinearOpMode{



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
        double LeftstickY;
        double RightstickX;
        boolean GripperState = true;
        double GripperToggle;
        double GripperEO;
        double Rabs;
        double Labs;

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

            Gamepad currentGamepad1 = new Gamepad();
            Gamepad currentGamepad2 = new Gamepad();

            Gamepad previousGamepad1 = new Gamepad();
            Gamepad previousGamepad2 = new Gamepad();




// program has startedz
            waitForStart();

            while(opModeIsActive())   {

                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);

                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);


                //chasis

                boolean dpU = gamepad1.dpad_up;
                boolean dpD = gamepad1.dpad_down;
                boolean dpR1 = gamepad1.dpad_right;
                boolean dpL1 = gamepad1.dpad_left;

                ArmPosition = mA.getCurrentPosition() + PositionDiff;

                if(dpU && dpD == false){
                    LeftstickY = -0.8;
                }
                else if(dpD && dpU == false){
                    LeftstickY = .8;
                }
                else if(dpD == false && dpU == false) {
                    LeftstickY = gamepad1.left_stick_y;
                }
                else if(dpD && dpU){
                    LeftstickY = gamepad1.left_stick_y;
                }

                if(dpR1 && dpL1 == false){
                    RightstickX = .8;
                }
                else if(dpL1 && dpR1 == false){
                    RightstickX = -0.8;
                }
                else if(dpL1 == false && dpR1 == false) {
                    RightstickX = gamepad1.right_stick_x;
                }
                else if(dpL1 && dpR1){
                    RightstickX = gamepad1.right_stick_x;
                }

                mL.setPower((LeftstickY/1.5)-(RightstickX/2));
                mR.setPower((-LeftstickY/1.5)-(RightstickX/2));

                //gripper
                boolean RightTrigger = gamepad2.right_bumper;
                boolean RightDpad = gamepad2.dpad_right;


                GripperEO = GripperToggle % 2;

                if ((currentGamepad2.left_bumper && !previousGamepad2.left_bumper) || (currentGamepad2.right_bumper && !previousGamepad2.right_bumper)){
//                sG.setPosition(0.37);
//                GripperToggle ++;
                    GripperState = !GripperState;
                }
//            else if ((RightTrigger || LeftTrigger) && GripperEO != 0){
//              sG.setPosition(.6);
//              GripperToggle --;
//            }
//            else {
//            }

                if(GripperState){
                    sG.setPosition(.5);
                }
                else if(GripperState == false){
                    sG.setPosition(.65);
                }

                //wrist
                boolean rB2 = gamepad2.dpad_down;
                boolean lB2 = gamepad2.dpad_up;
//            boolean dpr2 = gamepad2.dpad_right;

                if(ArmPosition > 3650){
                    wristPosition = .85;
                }
                else {
                    if (RightDpad) {
                        wristPosition = .85;
                    }
           if (lB2){
                wristPosition = 1;

            }
                    else if (rB2) {
                        wristPosition = .7;
                    } else if (rB2 == false && lB2 == false) {

                    }
                }
                sW.setPosition(wristPosition);

                //arm

                double RightstickY2 = gamepad2.right_stick_y;
                double LeftstickY2 = gamepad2.left_stick_y;

                Rabs = Math.abs(RightstickY2);
                Labs = Math.abs(LeftstickY2);

                if(limit.isPressed() == false && ArmPosition >= 0){
                    if(Rabs > Labs){
                        mA.setPower((-RightstickY2)/2);
                    }
                    else {
                        mA.setPower((-LeftstickY2)/2);
                    }
//                mA.setPower((-RightstickY2)/2);
//                mA.setPower((-LeftstickY2)/2);
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
                    if(Rabs > Labs){
                        mA.setPower((-RightstickY2)/2);
                    }
                    else {
                        mA.setPower((-LeftstickY2)/2);
                    }

//                mA.setPower((-RightstickY2)/2);
//                mA.setPower((-LeftstickY2)/2);
                }
//            else if(FarBack = true){
//                mA.setPower(0.5);
//            }
//            else if(ArmPositzion >= 0){
//                FarBack = false;
//            }
                else if (limit.isPressed() == true){
                    mA.setPower(-.5);
                    ArmPosition = 3750;
                    PositionDiff = ArmPosition - mA.getCurrentPosition();
                }
//            if (ArmPosition >= 3850){
//                mA.setPower(-.5);
//            }



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


