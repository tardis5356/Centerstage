package org.firstinspires.ftc.teamcode.ARTEMIS.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Winch;

public class WinchPullUpCommand extends SequentialCommandGroup {

    public WinchPullUpCommand(Winch winch) {
        addCommands(
//            new InstantCommand(winch::retractScissor),
            new InstantCommand(winch::disablePWM),
//        new InstantCommand(winch::extendBraces),

            //new WaitCommand(1500),
            new InstantCommand(winch::liftRobot)
        );
    }
}
