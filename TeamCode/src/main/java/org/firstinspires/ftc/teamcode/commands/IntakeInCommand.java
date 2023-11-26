package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LEDs;

public class IntakeInCommand extends SequentialCommandGroup {
    public IntakeInCommand(Intake intake, LEDs leds) {
        addCommands(
                new InstantCommand(intake::in),
                new WaitCommand(3000),
                new InstantCommand(intake::stop),
                new InstantCommand(() ->
                        leds.setLEDstate("Intaking")
                )
        );
    }
}
