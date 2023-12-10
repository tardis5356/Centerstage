package org.firstinspires.ftc.teamcode.ARTEMIS.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Arm;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Intake;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.LEDs;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Lift;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Winch;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Wrist;

public class RobotToStateCommand extends ParallelCommandGroup {

    public String currentState = "";

    public RobotToStateCommand(Arm arm, Wrist wrist, Gripper gripper, Lift lift, Intake intake, Winch winch, LEDs leds, String state) {
        switch (state.toLowerCase()) {
            case "intake":
                currentState = "intake";
                addCommands(
                        new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                        // send lift to fully retracted, wait for it to reach that position (via isFinished)
                                        new InstantCommand(intake::stop),
                                        // make sure arm isn't already in intake
                                        new WaitUntilCommand(() -> !arm.inIntakeEntering()),

                                        // open grippers & send wrist to transition position
                                        new InstantCommand(wrist::toTransition),
                                        new InstantCommand(gripper::releaseRight),
                                        new InstantCommand(gripper::releaseLeft),

                                        // wait 1 second for servos to move
                                        new WaitCommand(500),

                                        // move arm to transition position
                                        new InstantCommand(arm::toTransition),

                                        // wait for arm to reach transition position
                                        new WaitUntilCommand(() -> arm.inIntakeEntering()),
//                                new WaitUntilCommand(() -> arm.fullIntake()),
                                        new WaitCommand(100),

                                        // send wrist to intake position
                                        new InstantCommand(wrist::tiltToIntake),

                                        // wait for wrist to catch up
                                        new WaitCommand(200),

                                        // send arm to intake position
                                        new InstantCommand(arm::toIntake)
                                ),
                                new LiftToPositionCommand(lift, -10, 25)
                                )
                );
                break;
            case "grab_pixels":
                currentState = "grab_pixels";
                addCommands(
                        new SequentialCommandGroup(
                                // ensure arm is already in intake
                                new WaitUntilCommand(() -> arm.inIntakeFully()),

//                                new IntakeInCommand(intake, leds),
                                new InstantCommand(intake::in),

                                // send arm to grabbing position
                                new InstantCommand(arm::toGrab),

                                // wait .75 seconds
                                new WaitCommand(250),

                                // grab with both grippers
                                new InstantCommand(gripper::grabLeft),
                                new InstantCommand(gripper::grabRight),

                                // wait 1 second for grippers to grab
//                                new WaitCommand(250),

                                new IntakeOutCommand(intake),

                                // move arm & wrist to transition position
                                new InstantCommand(arm::toTransition),
                                new InstantCommand(wrist::toTransition)
                        )
                );
                break;
            case "deposit":
                currentState = "deposit";
                addCommands(
//                        new IntakeOutCommand(intake),
                        new InstantCommand(intake::out),
                        new SequentialCommandGroup(
                                // set arm to transition position (should already be there)
                                new InstantCommand(arm::toTransition),
                                new InstantCommand(wrist::toTransition),

                                new WaitCommand(250),

                                // ensure arm isn't in intake anymore
                                new WaitUntilCommand(() -> !arm.inIntakeExiting()),

                                // wait for one second
//                                new WaitCommand(1000),

                                // send arm && lift to deposit
                                // TODO: tune lift pid
                                new InstantCommand(arm::toDeposit),
                                //new LiftToPositionCommand(lift, 100, 25),

                                new WaitCommand(250),

//                                new WaitCommand(100),
                                new InstantCommand(wrist::tiltToDeposit),

                                // wait for .5 seconds for arm to move
                                new WaitCommand(500),

                                // tilt wrist to deposit
                                new InstantCommand(intake::stop)
                        )
                );
                break;
            case "dropPurple":
                currentState = "dropPurple";
                addCommands(
                        new IntakeInCommand(intake, leds),
                        new SequentialCommandGroup(
                                // set arm to transition position (should already be there)
                                new InstantCommand(arm::toTransition),
                                new InstantCommand(wrist::toTransition),

                                // ensure arm isn't in intake anymore
                                new WaitUntilCommand(() -> !arm.inIntakeExiting()),

                                // wait for one second
//                                new WaitCommand(1000),

                                // send arm && lift to deposit
                                // TODO: tune lift pid
                                new InstantCommand(arm::toDropPurple),
                                new LiftToPositionCommand(lift, 100, 25),

                                // wait for .5 seconds for arm to move
                                new WaitCommand(100),

                                // tilt wrist to deposit
                                new InstantCommand(wrist::tiltToDropPurplePixel)
                        )
                );
                break;
        }
    }
}
