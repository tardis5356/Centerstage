package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeOutCommand extends SequentialCommandGroup {
    public IntakeOutCommand(Intake intake) {
        addCommands(
                new InstantCommand(intake::out),
                new WaitCommand(1500),
                new InstantCommand(intake::stop)
        );
    }
}
