package org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands.IntakeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
//import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Intake;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.LEDs;

public class IntakeIn extends SequentialCommandGroup {
    public IntakeIn(Intake intake, LEDs leds) {
        addCommands(
                new InstantCommand(intake::in),
                new WaitCommand(1500),
                new InstantCommand(intake::stop),
                new InstantCommand(() -> {
                    leds.setLEDstate("Intaking");
                })
        );
    }
}
