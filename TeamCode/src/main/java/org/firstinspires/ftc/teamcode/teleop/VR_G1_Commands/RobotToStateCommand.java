package org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Arm;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Intake;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.LEDs;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Lift;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Winch;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems.Wrist;

public class RobotToStateCommand extends ParallelCommandGroup {

    public RobotToStateCommand(Arm arm, Wrist wrist, Gripper gripper, Lift lift, Intake intake, Winch winch, LEDs leds, String state) {
        switch (state) {
            case "intake":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(wrist::toTransition),
                                new InstantCommand(gripper::releaseRight),
                                new InstantCommand(gripper::releaseLeft),
                                new WaitCommand(1000),
                                new InstantCommand(arm::toIntake),
                                new WaitCommand(500),
                                new InstantCommand(wrist::tiltToIntake)
                        )
                );
                break;
            case "deposit":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(arm::toDeposit),
                                new InstantCommand(wrist::toTransition),
                                new WaitCommand(1000),
                                new InstantCommand(wrist::tiltToDeposit)
                        )
                );
                break;
        }
    }
}
