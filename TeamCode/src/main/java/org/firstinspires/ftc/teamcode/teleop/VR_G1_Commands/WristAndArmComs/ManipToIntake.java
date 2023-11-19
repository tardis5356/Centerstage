package org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands.WristAndArmComs;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Arm;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Wrist;

public class ManipToIntake extends SequentialCommandGroup {

    public ManipToIntake (Wrist wrist, Arm arm, Gripper gripper){
        addCommands(
            new InstantCommand(wrist::toTransition),
            new InstantCommand(gripper::releaseRight),
            new InstantCommand(gripper::releaseLeft),
            new WaitCommand(1000),
            new InstantCommand(arm::toTransition),
            new InstantCommand(wrist::tiltToIntake),
            new WaitCommand(500),
            new InstantCommand(arm::toIntake)
        );


    }
}
