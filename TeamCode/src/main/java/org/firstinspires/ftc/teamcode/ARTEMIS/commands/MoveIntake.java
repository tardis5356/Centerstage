package org.firstinspires.ftc.teamcode.ARTEMIS.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Intake;

public class MoveIntake extends ParallelCommandGroup {
    public MoveIntake(Intake intake, String state) {
        switch (state.toLowerCase()) {
            case "down":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(intake::up),
                                new WaitCommand(500)
//                                        new InstantCommand(intake:)
                        )
                );
                break;
        }
    }
}
