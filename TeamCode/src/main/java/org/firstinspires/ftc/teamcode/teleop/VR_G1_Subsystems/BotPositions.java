package org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
//This class has all the servo and PID motor positions.
//The main idea is that when you change a variable you only have to here.
//To use variables in other classes, import BotPositions, make an instance of it within the subsystem you're working in,
//and whenever you want to use the variable, use the instance name .variable name that you assign.
public class BotPositions {
    //winch powers and positions
    public static double WINCH_MOTOR_POWER = .8, WINCH_SERVO_DEPLOYED_POSITION = .2, WINCH_SERVO_RETRACTED_POSITION = .6;

    //brace positions
    public static double LEFT_BRACE_POSITION = .8, RIGHT_BRACE_POSITION = .8;

    //intake powers and positions
    public static double INTAKE_MOTOR_INWARD_POWER = -.8, INTAKE_MOTOR_OUTWARD_POWER = .3;
    //the stationary intake power is 0, what did you think it'd be?

    //gripper positions
    public static double LEFT_GRIPPER_CLOSED_POSITION = .9, LEFT_GRIPPER_OPEN_POSITION = .3, RIGHT_GRIPPER_CLOSED_POSITION = .2, RIGHT_GRIPPER_OPEN_POSITION = .8;

    //lift pid variables
    public static double LIFT_p = 0, LIFT_i = 0, LIFT_d = 0, LIFT_ff = .22;
    public static int LIFT_TOLERANCE = 25;
}
