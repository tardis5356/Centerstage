package org.firstinspires.ftc.teamcode.DemoBots.optimus.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.DemoBots.optimus.Optimus_subsystems.OptimusDrive;

public class OptimusDriveBackwardsCommand extends SequentialCommandGroup {
    public OptimusDriveBackwardsCommand(OptimusDrive drive, int timeMS) {
        addCommands(
                new InstantCommand(drive::DriveBackward),
                new WaitCommand(timeMS),
                new InstantCommand(drive::stopDriving)
        );
    }
}
