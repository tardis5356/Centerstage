package org.firstinspires.ftc.teamcode.commands.WristAndArmComs;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class ManipToIntake extends SequentialCommandGroup {

    public ManipToIntake (Wrist wrist, Arm arm, Gripper gripper){
        addCommands(
            new InstantCommand(wrist::toTransition),
            new InstantCommand(gripper::releaseRight),
            new InstantCommand(gripper::releaseLeft),
            new WaitCommand(1000),
            new InstantCommand(arm::toTransition),
            new InstantCommand(wrist::tiltToIntake),
            new WaitCommand(500),
            new InstantCommand(arm::toIntake)
        );


    }
}
