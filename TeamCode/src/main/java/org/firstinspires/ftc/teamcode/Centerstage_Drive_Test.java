//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions;
//import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Gripper;
//
////@Disabled
//@TeleOp (name = "Centerstage_Drive_Test")
//public class Centerstage_Drive_Test extends LinearOpMode {
//
//    double FB;
//    double LR;
//    double Rotation;
//    Servo LeftGrip, RightGrip, Roll;
//
//
//    DcMotor mFL, mFR, mBL, mBR;
//    Servo Drone;
//
//    @Override
//    public void runOpMode() {
//
//        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
//        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
//        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
//        mBR = hardwareMap.get(DcMotorEx.class, "mBR");
//        //Drone = hardwareMap.get(Servo.class, "Drone");
//
//        //LeftGrip = hardwareMap.get(Servo.class, "LG");
//        //RightGrip = hardwareMap.get(Servo.class, "RG");
//        Roll = hardwareMap.get(Servo.class, "Roll");
//
//
//        mBR.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            FB = gamepad1.left_stick_y;
//            LR = -gamepad1.left_stick_x;
//            Rotation = -gamepad1.right_stick_x;
//
//            mFL.setPower(FB+LR+Rotation);
//            mFR.setPower(FB-LR-Rotation);
//            mBL.setPower(FB-LR+Rotation);
//            mBR.setPower(FB+LR-Rotation);
//
//            if (gamepad1.b == true){
//                Drone.setPosition(Test_Bed_Drone_Positions.Launched);
//            }
//            else{
//                Drone.setPosition(Test_Bed_Drone_Positions.Latched);
//            }
//
//            if (gamepad1.dpad_up == true){
//                Roll.setPosition(BotPositions.WRIST_ROLL_CENTERED);
//                LeftGrip.setPosition(BotPositions.GRIPPER_LEFT_CLOSED);
//                RightGrip.setPosition(BotPositions.GRIPPER_RIGHT_CLOSED);
//            }
//            else if (gamepad1.dpad_down == true){
//                LeftGrip.setPosition(BotPositions.GRIPPER_LEFT_OPEN);
//                RightGrip.setPosition(BotPositions.GRIPPER_RIGHT_OPEN);
//            }
//
//        }
//
//    }
//
//}
