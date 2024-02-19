package org.firstinspires.ftc.teamcode.DemoBots.optimus.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.DemoBots.optimus.Optimus_subsystems.Gripper;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.Optimus_subsystems.Wrist;

public class OptimusRobotToIntakePositionCommand extends SequentialCommandGroup {

    public OptimusRobotToIntakePositionCommand(Gripper gripper, Wrist wrist){
        addCommands(
                new InstantCommand(gripper::open),
                new WaitCommand(500),
                new InstantCommand(wrist::intake)
        );
    }
}
