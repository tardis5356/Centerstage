package org.firstinspires.ftc.teamcode.ARTEMIS.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
//This class has all the servo and PID motor positions.
//The main idea is that when you change a variable you only have to here.
//To use variables in other classes, import BotPositions, make an instance of it within the subsystem you're working in,
//and whenever you want to use the variable, use the instance name .variable name that you assign.
public class BotPositions {
    //winch powers and positions
    public static double WINCH_MOTOR_POWER = 0.8, WINCH_SERVO_DEPLOYED = 0.35, WINCH_SERVO_RETRACTED = 0.6;

    //brace positions
    public static double LEFT_BRACE_EXTENDED = 0.69, RIGHT_BRACE_EXTENDED = 0.68, RIGHT_BRACE_RETRACTED = 0.92, LEFT_BRACE_OVEREXTENDED = 0.28, RIGHT_BRACE_OVEREXTENDED = 0.35;

    //intake powers and positions
    public static double INTAKE_MOTOR_INWARD_POWER = 0.8, INTAKE_MOTOR_OUTWARD_POWER = -.6, INTAKE_MOTOR_OUTWARD_POWER_SLOW = -.25, INTAKE_MOTOR_OUTWARD_POWER_SUPER_SLOW = -.2;

    // TORQUE POSITTIOOUNS
    public static double
            INTAKE_DOWN_TELEOP = 0.45,
            INTAKE_DOWN_FIRST_PIXEL = 0.25,
            INTAKE_DOWN_SECOND_PIXEL = 0.3,
            INTAKE_DOWN_THIRD_PIXEL = 0.35,
            INTAKE_DOWN_FOURTH_PIXEL = 0.4,
            INTAKE_DOWN_FIFTH_PIXEL = 0.45,
            INTAKE_UP = 0.16; // torque: 0.5 down, 0.25 up // 0.65

    /*
    // SAVOX POSITTIOOUNS
    public static double
            INTAKE_DOWN_TELEOP = 0.2,
            INTAKE_DOWN_FIRST_PIXEL = 0.39,
            INTAKE_DOWN_SECOND_PIXEL = 0.4,
            INTAKE_DOWN_THIRD_PIXEL = 0.35,
            INTAKE_DOWN_FOURTH_PIXEL = 0.3,
            INTAKE_DOWN_FIFTH_PIXEL = 0.25,
            INTAKE_UP = 0.67; // torque: 0.5 down, 0.25 up // 0.65
     */


    //gripper positions
    public static double GRIPPER_LEFT_CLOSED = 0.79, GRIPPER_LEFT_OPEN = 0.33, GRIPPER_RIGHT_CLOSED = 0.22, GRIPPER_RIGHT_OPEN = 0.65;

    //lift pid variables
    public static double LIFT_p = 0.02, LIFT_i = 0, LIFT_d = 0, LIFT_ff = -.09;
    public static int LIFT_TOLERANCE = 25;
    public static double DISTANCE_FROM_BACKDROP = 20, DISTANCE_FROM_BACKDROP_TOLERANCE;

    //wrist positions
    public static double
//            WRIST_LEFT_pos90_ROLL = 0.561, // og vert? don't think it was actually used
            //            WRIST_LEFT_pos30_ROLL = 0.497, // og tilt left
            WRIST_LEFT_pos180_ROLL = 0.58,
            WRIST_LEFT_pos150_ROLL = 0.544,
            WRIST_LEFT_pos120_ROLL = 0.54,
            WRIST_LEFT_pos90_ROLL = 0.512,
            WRIST_LEFT_pos60_ROLL = 0.496,
            WRIST_LEFT_pos30_ROLL = 0.48,
            WRIST_ROLL_CENTERED = 0.464,
    //            WRIST_RIGHT_neg30_ROLL = 0.432,
            WRIST_RIGHT_neg30_ROLL = 0.448,
            WRIST_RIGHT_neg60_ROLL = 0.43,
            WRIST_RIGHT_neg90_ROLL = 0.416,
            WRIST_RIGHT_neg120_ROLL = 0.385,
            WRIST_RIGHT_neg150_ROLL = 0.384,
            WRIST_RIGHT_neg180_ROLL = 0.345,
            WRIST_TILT_TRANSITION = 0.19, WRIST_TILT_INTAKE = 0.29, WRIST_TILT_DEPOSIT = 0.78, WRIST_TILT_DEPOSIT_LOW = 0.68, WRIST_TILT_DROP_PURPLE = 0.8;
    //for tilt, 0.09 = 30º
//    0.115 = 30º

    //0.016 = 30º

    //0.032 = 60º
    //0.032 = 90º
    //0.032 = 120º
    //0.032 = 180º

    //arm positions
    public static double ARM_INTAKE = 0.025, ARM_TRANSITION_POSITION = 0.12, ARM_DEPOSIT = 0.65, ARM_DEPOSIT_LOW = 0.75, ARM_DROP_PURPLE = 0.9, ARM_GRAB_PIXELS = .025;

    public static boolean ARM_HIGH_POSITION = false;


    //drone servo positions
    public static double DRONE_LATCHED = 0.36, DRONE_UNLATCHED = 0.6;

    public static double BACK_WEBCAM_Y_OFFSET = 0, BACK_WEBCAM_X_OFFSET = 8; // x is front to back, y is side to side
    // 7.5
}
