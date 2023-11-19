package org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands.WinchCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Winch;

public class DeployWinch extends SequentialCommandGroup {
    public DeployWinch(Winch winch) {
        addCommands(
                new InstantCommand(winch::extendScissor),
                new InstantCommand(winch::extendBraces),
                new InstantCommand(winch::extendBraces)
        );

    }
}
