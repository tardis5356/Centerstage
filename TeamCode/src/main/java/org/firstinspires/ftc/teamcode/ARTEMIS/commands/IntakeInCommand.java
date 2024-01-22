package org.firstinspires.ftc.teamcode.ARTEMIS.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Intake;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.LEDs;

public class IntakeInCommand extends SequentialCommandGroup {
    public IntakeInCommand(Intake intake, LEDs leds) {
        addCommands(
                new InstantCommand(() ->
                        leds.setLEDstate("Intaking")
                ),
                new InstantCommand(intake::in),
                new WaitCommand(5000),
                new InstantCommand(intake::stop)
        );
    }
}
