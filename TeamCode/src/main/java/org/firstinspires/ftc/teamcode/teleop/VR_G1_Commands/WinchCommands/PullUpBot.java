package org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands.WinchCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Winch;

public class PullUpBot extends SequentialCommandGroup {

    public PullUpBot (Winch winch) {
        addCommands(
            new InstantCommand(winch::retractScissor),
            new InstantCommand(winch::extendBraces),
            new WaitCommand(1500),
            new InstantCommand(winch::liftRobot)
        );
    }
}
