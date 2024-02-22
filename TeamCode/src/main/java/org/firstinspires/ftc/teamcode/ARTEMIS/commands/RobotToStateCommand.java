package org.firstinspires.ftc.teamcode.ARTEMIS.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.ARTEMIS.commands.intakeCommands.IntakeOutCommand;
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
//                                        new WaitUntilCommand(() -> !arm.inIntakeEntering()),

                                        // open grippers & send wrist to transition position
                                        new InstantCommand(wrist::toTransition),
                                        new InstantCommand(gripper::releaseRight),
                                        new InstantCommand(gripper::releaseLeft),

                                        // wait 1 second for servos to move
                                        new WaitCommand(250),

                                        // move arm to transition position
                                        new InstantCommand(arm::toTransition),

                                        // wait for arm to reach transition position
//                                        new WaitUntilCommand(() -> arm.inIntakeEntering()),
//                                new WaitUntilCommand(() -> arm.fullIntake()),
                                        new WaitCommand(400),

                                        // send wrist to intake position
                                        new InstantCommand(wrist::tiltToIntake),

                                        // wait for wrist to catch up
                                        new WaitCommand(50),

                                        // send arm to intake position
                                        new InstantCommand(arm::toIntake)
                                ),
                                new LiftToPositionCommand(lift, -10, 25)
                        )
                );
                break;
            case "intakenorelease":
                currentState = "intakenorelease";
                addCommands(
                        new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                        // send lift to fully retracted, wait for it to reach that position (via isFinished)
                                        new InstantCommand(intake::stop),
                                        // make sure arm isn't already in intake
//                                        new WaitUntilCommand(() -> !arm.inIntakeEntering()),

                                        // open grippers & send wrist to transition position
                                        new InstantCommand(wrist::toTransition),

                                        // wait 1 second for servos to move
                                        new WaitCommand(250),

                                        // move arm to transition position
                                        new InstantCommand(arm::toTransition),

                                        // wait for arm to reach transition position
//                                        new WaitUntilCommand(() -> arm.inIntakeEntering()),
//                                new WaitUntilCommand(() -> arm.fullIntake()),
                                        new WaitCommand(400),

                                        // send wrist to intake position
                                        new InstantCommand(wrist::tiltToIntake),

                                        // wait for wrist to catch up
                                        new WaitCommand(50),

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
//                                new InstantCommand(arm::toGrab),

                                // wait .75 seconds
//                                new WaitCommand(250),

                                // grab with both grippers
//                                new InstantCommand(gripper::grabLeft),
//                                new InstantCommand(gripper::grabRight),

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
                        new SequentialCommandGroup(
//                                new InstantCommand(intake::in),
                                // set arm to transition position (should already be there)
                                new InstantCommand(arm::toTransition),

                                new WaitCommand(100),

//                                new InstantCommand(intake::out),
                                new InstantCommand(wrist::toTransition),

                                new WaitCommand(300),

                                // ensure arm isn't in intake anymore
//                                new WaitUntilCommand(() -> !arm.inIntakeExiting()),

                                // wait for one second
//                                new WaitCommand(100),

                                // send arm && lift to deposit
                                new InstantCommand(arm::toDeposit),
                                new InstantCommand(intake::out),
                                //new LiftToPositionCommand(lift, 100, 25),

                                new WaitCommand(150),

//                                new WaitCommand(100),
                                new InstantCommand(wrist::tiltToDeposit),

                                // wait for .5 seconds for arm to move
                                new WaitCommand(250),

                                // tilt wrist to deposit
                                new InstantCommand(intake::stop)
                        )
                );
                break;
            case "deposithigh":
                currentState = "depositHigh";
                addCommands(
//                        new IntakeOutCommand(intake),
                        new SequentialCommandGroup(
//                                new InstantCommand(intake::in),
                                // set arm to transition position (should already be there)
                                new InstantCommand(arm::toTransition),

                                new WaitCommand(100),

//                                new InstantCommand(intake::out),
                                new InstantCommand(wrist::toTransition),

                                new WaitCommand(300),

                                // ensure arm isn't in intake anymore
//                                new WaitUntilCommand(() -> !arm.inIntakeExiting()),

                                // wait for one second
//                                new WaitCommand(100),

                                // send arm && lift to deposit
                                new InstantCommand(arm::toDepositHigh),
                                new InstantCommand(intake::out),
                                //new LiftToPositionCommand(lift, 100, 25),

                                new WaitCommand(150),

//                                new WaitCommand(100),
                                new InstantCommand(wrist::tiltToDepositHigh),

                                // wait for .5 seconds for arm to move
                                new WaitCommand(250),

                                // tilt wrist to deposit
                                new InstantCommand(intake::stop)
                        )
                );
                break;
            case "droppurple":
                currentState = "dropPurple";
                addCommands(
//                        new IntakeOutCommand(intake),
                        new SequentialCommandGroup(
//                                new InstantCommand(intake::in),
                                // set arm to transition position (should already be there)
                                new InstantCommand(arm::toTransition),

                                new WaitCommand(100),

//                                new InstantCommand(intake::out),
                                new InstantCommand(wrist::toTransition),

                                new WaitCommand(300),

                                // ensure arm isn't in intake anymore
//                                new WaitUntilCommand(() -> !arm.inIntakeExiting()),

                                // wait for one second
//                                new WaitCommand(100),

                                // send arm && lift to deposit
                                new InstantCommand(arm::toDropPurple),
                                new InstantCommand(intake::out),
                                new LiftToPositionCommand(lift, 100, 25),

                                new WaitCommand(150),

//                                new WaitCommand(100),
                                new InstantCommand(wrist::tiltToDropPurplePixel),

                                // wait for .5 seconds for arm to move
                                new WaitCommand(250),

                                // tilt wrist to deposit
                                new InstantCommand(intake::stop)
                        )
                );
                break;
        }
    }
}
