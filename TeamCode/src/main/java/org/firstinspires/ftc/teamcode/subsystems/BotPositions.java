package org.firstinspires.ftc.teamcode.subsystems;

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
    public static double LEFT_BRACE_EXTENDED = 0.79, RIGHT_BRACE_EXTENDED = 0.7;

    //intake powers and positions
    public static double INTAKE_MOTOR_INWARD_POWER = 1, INTAKE_MOTOR_OUTWARD_POWER = -.8;
    //the stationary intake power is 0, what did you think it'd be?

    //gripper positions
    public static double GRIPPER_LEFT_CLOSED = 0.79, GRIPPER_LEFT_OPEN = 0.4, GRIPPER_RIGHT_CLOSED = 0.22, GRIPPER_RIGHT_OPEN = 0.6;

    //lift pid variables
    public static double LIFT_p = 0, LIFT_i = 0, LIFT_d = 0, LIFT_ff = -.22;
    public static int LIFT_TOLERANCE = 25;

    //wrist positions
    public static double WRIST_LEFT_ROLL = 0.72, WRIST_RIGHT_ROLL = 0.27, WRIST_ROLL_CENTERED = 0.49, WRIST_TILT_TRANSITION = 0.79, WRIST_TILT_INTAKE = 0.767, WRIST_TILT_DEPOSIT = 0.665;
    //for tilt, 0.09 = 30º
//    0.115 = 30º

    //arm positions
    public static double ARM_INTAKE = 0.13, ARM_TRANSITION_POSITION = 0.2,  ARM_DEPOSIT = 0.67, ARM_GRAB_PIXELS = .06;

    //drone servo positions
    public static double DRONE_LATCHED = 0.33, DRONE_UNLATCHED = 0.27;
}
