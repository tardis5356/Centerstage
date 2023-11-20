package org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands.IntakeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Intake;

public class IntakeOut extends SequentialCommandGroup {
    public IntakeOut(Intake intake) {
        addCommands(
                new InstantCommand(intake::out),
                new WaitCommand(1500),
                new InstantCommand(intake::stop)
        );
    }
}
