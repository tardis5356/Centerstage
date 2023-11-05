package org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands.WinchComs;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Winch;

public class DeployWinch extends SequentialCommandGroup{

    public DeployWinch (Winch winch){

        addCommands(
                new InstantCommand(winch :: scissorDep),
                new InstantCommand(winch :: braceDep),
                new InstantCommand(winch :: braceDep)
        );

    }
}
