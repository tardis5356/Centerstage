package org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
//This class has all the servo and PID motor positions.
//The main idea is that when you change a variable you only have to here.
//To use variables in other classes, import BotPositions, make an instance of it within the subsystem you're working in,
//and whenever you want to use the variable, use the instance name .variable name that you assign.
public class BotPositions {
    //winch powers and positions
    public static double WINCH_MOTOR_POWER = .8, WINCH_SERVO_DEPLOYED = .2, WINCH_SERVO_RETRACTED = .6;

    //brace positions
    public static double LEFT_BRACE_EXTENDED = .8, RIGHT_BRACE_EXTENDED = .8;

    //intake powers and positions
    public static double INTAKE_MOTOR_INWARD_POWER = -.8, INTAKE_MOTOR_OUTWARD_POWER = .3;
    //the stationary intake power is 0, what did you think it'd be?

    //gripper positions
    public static double GRIPPER_LEFT_CLOSED = 0.79, GRIPPER_LEFT_OPEN = .4, GRIPPER_RIGHT_CLOSED = .22, GRIPPER_RIGHT_OPEN = .6;

    //lift pid variables
    public static double LIFT_p = 0, LIFT_i = 0, LIFT_d = 0, LIFT_ff = .22;
    public static int LIFT_TOLERANCE = 25;

    //wrist positions
    public static double WRIST_LEFT_ROLL = .72, WRIST_RIGHT_ROLL = 0.27, WRIST_ROLL_CENTERED = 0.49, WRIST_TILT_TRANSITION = 0.785, WRIST_TILT_INTAKE = .77, WRIST_TILT_DEPOSIT = 0.665;
    //for tilt, 0.09 = 30º
//    0.115 = 30º

    //arm positions
    public static double ARM_LEFT_INTAKE = .8, ARM_RIGHT_INTAKE = .2, ARM_LEFT_TRANSITION_POSITION = .65, ARM_RIGHT_TRANSITION_POSITION = .35, ARM_LEFT_DEPOSIT = .3, ARM_RIGHT_DEPOSIT = .7;

    //drone servo positions
    public static double DRONE_LATCHED = .3, DRONE_UNLATCHED = .2;
}
