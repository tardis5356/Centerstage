package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Winch;

public class WinchDeployCommand extends SequentialCommandGroup {
    public WinchDeployCommand(Winch winch) {
        addCommands(
                new InstantCommand(winch::extendScissor)//,
                //new InstantCommand(winch::extendBraces),
                //new InstantCommand(winch::extendBraces)
        );

    }
}
