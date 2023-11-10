package org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands.IntakeComs;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Intake;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.LEDs;

public class IntakeIn extends SequentialCommandGroup {
    public IntakeIn(Intake intake, LEDs led){
        addCommands(
                new InstantCommand(intake :: in),
                new InstantCommand(intake :: stop),
                new InstantCommand(()->{
                    LEDs.setLEDstate("Intaking");
                })

        );
    }
}
