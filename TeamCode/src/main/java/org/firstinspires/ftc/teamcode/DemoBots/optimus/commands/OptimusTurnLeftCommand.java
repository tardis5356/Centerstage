package org.firstinspires.ftc.teamcode.DemoBots.optimus.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.DemoBots.optimus.Optimus_subsystems.OptimusDrive;

public class OptimusTurnLeftCommand extends SequentialCommandGroup {
    public OptimusTurnLeftCommand(OptimusDrive drive){
        addCommands(
                new InstantCommand(drive :: TurnLeft)

        );
    }
}
