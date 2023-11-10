package org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands.GripperComs;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Gripper;

public class OpenLeftGripper extends SequentialCommandGroup {
    public OpenLeftGripper(Gripper gripper){
        addCommands(
                new InstantCommand(gripper::releaseLeft)
        );
    }
}