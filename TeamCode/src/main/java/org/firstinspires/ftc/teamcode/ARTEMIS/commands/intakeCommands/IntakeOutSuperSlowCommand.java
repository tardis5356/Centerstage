package org.firstinspires.ftc.teamcode.ARTEMIS.commands.intakeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Intake;

public class IntakeOutSuperSlowCommand extends SequentialCommandGroup {
    public IntakeOutSuperSlowCommand(Intake intake) {
        addCommands(
                new InstantCommand(intake::slowOut),
                new WaitCommand(750),
                new InstantCommand(intake::stop)
        );
    }
}
