package org.firstinspires.ftc.teamcode.ARTEMIS.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Intake;

public class IntakeOutSlowCommand extends SequentialCommandGroup {
    public IntakeOutSlowCommand(Intake intake) {
        addCommands(
                new InstantCommand(intake::slowOut),
                new WaitCommand(750),
                new InstantCommand(intake::stop)
        );
    }
}
