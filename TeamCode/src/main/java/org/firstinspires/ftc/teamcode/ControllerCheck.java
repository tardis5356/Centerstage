package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name="ControllerCheck")
public class ControllerCheck extends LinearOpMode{
        @Override
    public void runOpMode (){


            waitForStart();
            while(opModeIsActive()){

                    boolean LeftBumper = gamepad1.left_bumper;
                    boolean RightBumper = gamepad1.right_bumper;

                    double LeftTrigger = gamepad1.left_trigger;
                    double RightTrigger = gamepad1.right_trigger;

                    boolean Back = gamepad1.back;
                    boolean Start = gamepad1.start;

                    boolean Up = gamepad1.dpad_up;
                    boolean Down = gamepad1.dpad_down;
                    boolean Left = gamepad1.dpad_left;
                    boolean Right = gamepad1.dpad_right;

                    boolean A = gamepad1.a;
                    boolean B = gamepad1.b;
                    boolean X = gamepad1.x;
                    boolean Y = gamepad1.y;

                    boolean LeftStickButton = gamepad1.left_stick_button;
                    boolean RightStickButton = gamepad1.right_stick_button;
                    double LeftStickX = gamepad1.left_stick_x;
                    double LeftStickY = gamepad1.left_stick_y;
                    double RightStickX = gamepad1.right_stick_x;
                    double RightStickY = gamepad1.right_stick_y;

            telemetry.addData("Right_Bumper", RightBumper);
            telemetry.addData("Left_Bumper", LeftBumper);

            telemetry.addData("Right_Trigger", RightTrigger);
            telemetry.addData("Left_Trigger", LeftTrigger);

            telemetry.addData("Up_Dpad", Up);
            telemetry.addData("Down_Dpad", Down);
            telemetry.addData("Left_Dpad", Left);
            telemetry.addData("Right_Dpad", Right);

            telemetry.addData("A", A);
            telemetry.addData("B", B);
            telemetry.addData("X", X);
            telemetry.addData("Y", Y);

            telemetry.addData("Left_Stick_Button", LeftStickButton);
            telemetry.addData("Right_Stick_Button", RightStickButton);

            telemetry.addData("Left_Stick_X", LeftStickX);
            telemetry.addData("Left_Stick_Y", LeftStickY);

            telemetry.addData("Right_Stick_X", RightStickX);
            telemetry.addData("Right_Stick_Y", RightStickY);

            telemetry.update();
            }
        }
}
