package org.firstinspires.ftc.teamcode.DemoBots.primus.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.DemoBots.primus.Primus_subsystems.Gripper;

public class CloseGripper extends SequentialCommandGroup{

    public CloseGripper(Gripper left, Gripper right){
        addCommands(
                new InstantCommand(left::close),
                new InstantCommand(right::close),
                new WaitCommand(500)
        );
    }
}
