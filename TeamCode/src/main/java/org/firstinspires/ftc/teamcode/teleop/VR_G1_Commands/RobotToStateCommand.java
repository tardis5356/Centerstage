package org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands.IntakeCommands.IntakeIn;
import org.firstinspires.ftc.teamcode.teleop.VR_G1_Commands.IntakeCommands.IntakeOut;
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
                                new LiftToPositionCommand(lift, -10, 25),
                                new WaitUntilCommand(() -> arm.inIntake() == false),
                                new InstantCommand(wrist::toTransition),
                                new InstantCommand(gripper::releaseRight),
                                new InstantCommand(gripper::releaseLeft),
                                //new InstantCommand(lift::)
                                new WaitCommand(1000),
                                new InstantCommand(arm::toTransition),
                                new WaitUntilCommand(() -> arm.inIntake() == true),
                                new InstantCommand(wrist::tiltToIntake),
                                new WaitCommand(750),
                                new InstantCommand(arm::toIntake)
                        )
                );
                break;
            case "deposit":
                addCommands(
                        new IntakeIn(intake, leds),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(()->arm.inIntake() == true),
                                new InstantCommand(arm::toGrab),
                                new WaitCommand(750),
                                new InstantCommand(gripper::grabLeft),
                                new InstantCommand(gripper::grabRight),
                                new WaitCommand(1000),
                                new InstantCommand(arm::toTransition),
                                new InstantCommand(wrist::toTransition),
                                new WaitUntilCommand(()->arm.inIntake() == false),
                                //new InstantCommand(wrist::tiltToDeposit),
                                new WaitCommand(1000),
                                new InstantCommand(arm::toDeposit),
                                //new LiftToPositionCommand(lift, 100, 25),
                                new WaitCommand(1000),
                                new InstantCommand(wrist::tiltToDeposit)
                        )
                );
                break;
        }
    }
}
