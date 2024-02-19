package org.firstinspires.ftc.teamcode.DemoBots.optimus.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.DemoBots.optimus.Optimus_subsystems.OptimusDrive;

public class DriveBackwardsCommand extends SequentialCommandGroup {
    public DriveBackwardsCommand(OptimusDrive drive){
        addCommands(
                new InstantCommand(drive :: DriveBackward)

        );
    }
}
