package org.firstinspires.ftc.teamcode.ARTEMIS.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Winch;

public class WinchLatchCommand extends SequentialCommandGroup {
    public WinchLatchCommand(Winch winch) {
        addCommands(
                new InstantCommand(winch::latchHook),
                new InstantCommand(winch::latchBar)
        );
    }
}
