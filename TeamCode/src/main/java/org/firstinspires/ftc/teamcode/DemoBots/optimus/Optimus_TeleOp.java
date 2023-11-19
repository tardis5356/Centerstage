package org.firstinspires.ftc.teamcode.DemoBots.optimus;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Disabled
@TeleOp(name="Optimus_TeleOp")
public class Optimus_TeleOp extends LinearOpMode{

    // create objects and give them classes
    DcMotor mL;
    DcMotor mR;
    DcMotor mA;
    Servo sG;
    Servo sW;
    TouchSensor limit;
    double wristPosition = 0;
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
            double LeftstickY = gamepad1.left_stick_y;
            double RightstickX = gamepad1.right_stick_x;
            mL.setPower(LeftstickY-RightstickX);
            mR.setPower(-LeftstickY-RightstickX);

            //gripper
            double RightTrigger = gamepad2.right_trigger;

            if (RightTrigger != 0){
                sG.setPosition(0.2);
            } else {
              sG.setPosition(0.6);
            }

            //wrist
            boolean rB2 = gamepad2.right_bumper;
            boolean lB2 = gamepad2.left_bumper;

            if (rB2 && wristPosition < 1) {
                wristPosition += 0.01;
            }
            else if (lB2 && wristPosition >= 0){
                wristPosition -= 0.01;

            }
            else if (rB2 == false && lB2 == false){

            }

            sW.setPosition(wristPosition);

            //arm

            double RightstickY2 = gamepad2.right_stick_y;

           // if(limit.isPressed() == false){
            mA.setPower(-RightstickY2);
            //}

            //else if (limit.isPressed() == true){
           // mA.setPower(-0.5);

            //}


        }



    }
}
