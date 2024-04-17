package org.firstinspires.ftc.teamcode.ARTEMIS.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Winch;

public class HookDeployCommand extends SequentialCommandGroup {
    public HookDeployCommand(Winch winch) {
        addCommands(
                new InstantCommand(winch::unlatchHookStage1),
                new WaitCommand(100),
                new InstantCommand(winch::unlatchHookStage2),
                new WaitCommand(500),
                new InstantCommand(winch::latchHook)
        );
    }
}

