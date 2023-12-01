package org.firstinspires.ftc.teamcode.ARTEMIS.commands.WristAndArmComs;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Arm;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Wrist;

public class ManipToOutput extends SequentialCommandGroup {

    public ManipToOutput (Wrist wrist, Arm arm, Gripper gripper){
        addCommands(
                new InstantCommand(gripper::grabRight),
                new InstantCommand(gripper::grabLeft),
                new WaitCommand(500),
                new InstantCommand(arm::toTransition),
                new InstantCommand(wrist::toTransition),
                new WaitCommand(500),
                new InstantCommand(arm::toDeposit),
                new InstantCommand(wrist::tiltToDeposit)
        );
    }


}
