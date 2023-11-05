package org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands.WinchComs;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Winch;

public class PullUpBot extends SequentialCommandGroup {

    public PullUpBot (Winch winch) {
        addCommands(
            new InstantCommand(winch::retract),
            new WaitCommand(1500),
            new InstantCommand(winch::PullUp)
        );
    }
}
