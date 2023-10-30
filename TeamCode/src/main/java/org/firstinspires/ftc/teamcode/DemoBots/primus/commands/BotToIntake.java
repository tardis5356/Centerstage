package org.firstinspires.ftc.teamcode.DemoBots.primus.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.DemoBots.primus.Primus_subsystems.Gripper;

public class BotToIntake extends SequentialCommandGroup{

    public BotToIntake(Gripper left, Gripper right){
        addCommands(
                new InstantCommand(left::open),
                new InstantCommand(right::open),
                new WaitCommand(500)
        );
    }

}
