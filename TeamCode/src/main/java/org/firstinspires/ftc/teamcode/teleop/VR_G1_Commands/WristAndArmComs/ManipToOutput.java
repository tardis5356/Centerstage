package org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands.WristAndArmComs;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Arm;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Wrist;

public class ManipToOutput extends SequentialCommandGroup {

    public ManipToOutput (Wrist wrist, Arm arm, Gripper gripper){
        addCommands(
                new InstantCommand(gripper::grabRight),
                new InstantCommand(gripper::grabLeft),
                new WaitCommand(500),
                new InstantCommand(arm::ArmToIntakePrep),
                new InstantCommand(wrist::WristToIntakePrep),
                new WaitCommand(500),
                new InstantCommand(arm::ArmToOutPut),
                new InstantCommand(wrist::TiltToOutput)
        );
    }


}
