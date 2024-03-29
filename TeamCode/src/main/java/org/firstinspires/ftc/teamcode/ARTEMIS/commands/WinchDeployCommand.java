package org.firstinspires.ftc.teamcode.ARTEMIS.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Winch;

public class WinchDeployCommand extends SequentialCommandGroup {
    public WinchDeployCommand(Winch winch) {
        addCommands(
                new InstantCommand(winch::unlatchHook),
                new WaitCommand(500),
                new InstantCommand(winch::unlatchBar),
                new WaitCommand(500),
                new InstantCommand(winch::disablePWM)
        );
    }
}
