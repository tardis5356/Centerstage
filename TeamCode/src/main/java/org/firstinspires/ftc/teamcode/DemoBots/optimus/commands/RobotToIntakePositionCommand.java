package org.firstinspires.ftc.teamcode.DemoBots.optimus.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.DemoBots.optimus.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.subsystems.Wrist;

public class RobotToIntakePositionCommand extends SequentialCommandGroup {

    public RobotToIntakePositionCommand(Gripper gripper, Wrist wrist){
        addCommands(
                new InstantCommand(gripper::open),
                new WaitCommand(500),
                new InstantCommand(wrist::intake)
        );
    }
}
