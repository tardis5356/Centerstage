package org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands.IntakeComs;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Intake;

public class IntakeOut extends SequentialCommandGroup {
    public IntakeOut(Intake IntakeM){
        addCommands(
                new InstantCommand(IntakeM :: out),
                new WaitCommand(1500),
                new InstantCommand(IntakeM :: stop)
        );
    }
}
