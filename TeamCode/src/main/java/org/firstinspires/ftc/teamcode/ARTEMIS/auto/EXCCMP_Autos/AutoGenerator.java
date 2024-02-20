package org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos;

import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_SpikeToBackdrop;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_StackPickupSequence;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.accelConstraint40in;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.velConstraint10in;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.velConstraint20in;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ARTEMIS.auto.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.LiftToPositionCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.RobotAlignToTagRange;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.RobotToStateCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.autoCommands.AutoBackdropDepositCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.autoCommands.RelocalizeFromTagCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.intakeCommands.IntakeInCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.intakeCommands.IntakeOutCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Arm;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Intake;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.LEDs;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Lift;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Webcams;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Winch;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.ARTEMIS.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AutoGenerator {

    private SequentialCommandGroup autoCommands = new SequentialCommandGroup();

    public SequentialCommandGroup generateAutoCommands(Arm arm, Wrist wrist, Gripper gripper, Lift lift, Intake intake, Winch winch, LEDs leds, Drivetrain drivetrain, Webcams webcam, AprilTagMetadata targetBackdropTag, AprilTagProcessor aprilTagProcessor, SampleMecanumDrive drive, TrajectorySequence StartToSpike, TrajectorySequence SpikeToStack, TrajectorySequence StackToBack, TrajectorySequence BackToStack, TrajectorySequence SpikeToBackdrop, TrajectorySequence StackPickupSequence, String alliance, String startingSide, String cycleTarget, String transitVia, String parkIn, boolean cycle, boolean wait, boolean deliverYellow) {
        autoCommands.addCommands(new SequentialCommandGroup(
                new InstantCommand(() -> leds.setLEDstate("green")),
                new ParallelCommandGroup(
                        new FollowTrajectoryCommand(drive, StartToSpike),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "dropPurple")
                        )
                ),
                new WaitCommand(100),
                new InstantCommand(gripper::releaseLeft),
                new WaitCommand(200)
        ));

        // get extra pixel on first cycle (start: spike, end: back)
        if (startingSide == "wing") {
            autoCommands.addCommands(new SequentialCommandGroup(
                    new InstantCommand(() -> leds.setLEDstate("white")),
                    new WaitCommand(150),
                    new ParallelCommandGroup(
                            new FollowTrajectoryCommand(drive, SpikeToStack),
                            new SequentialCommandGroup(
                                    new WaitCommand(200),
                                    new ParallelDeadlineGroup(
                                            new WaitCommand(1000),
                                            new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds,
                                                    "intakeNoRelease")
                                    )
//                                    new InstantCommand(() -> leds.setLEDstate("purple"))
                            )
                    ),
//                    new InstantCommand(() -> leds.setLEDstate("yellow")),
                    new WaitCommand(100),
                    new InstantCommand(intake::in),
                    new InstantCommand(intake::down),
                    new ParallelCommandGroup(
                            new FollowTrajectoryCommand(drive, StackPickupSequence),
                            new SequentialCommandGroup(
                                    new WaitCommand(500),
                                    new InstantCommand(intake::out),
                                    new WaitCommand(200),
                                    new InstantCommand(intake::in),
                                    new WaitCommand(500),
                                    new InstantCommand(intake::out)
                            )
                    ),
//                    new ParallelCommandGroup(
//                            new IntakeInCommand(intake, leds),
//                            new FollowTrajectoryCommand(drive, StackPickupSequence)
//                    ),
                    new ParallelCommandGroup(
                            new InstantCommand(() -> leds.setLEDstate("yellow")),
                            new SequentialCommandGroup(
                                    new InstantCommand(gripper::grabLeft),
                                    new InstantCommand(gripper::grabRight),
                                    new InstantCommand(intake::up),
                                    new WaitCommand(150),
//                                    new IntakeOutCommand(intake)
                                    new InstantCommand(intake::out)
                            ),
                            new FollowTrajectoryCommand(drive, StackToBack)
                    )
            ));
        } else {
            autoCommands.addCommands(new SequentialCommandGroup( // turn bot around to align with backdrop
                    new FollowTrajectoryCommand(drive, SpikeToBackdrop)
            ));
        }

        // align to atag and deposit sequence
        if (deliverYellow)
            autoCommands.addCommands(new SequentialCommandGroup(
                    new InstantCommand(intake::stop),

                    new InstantCommand(() -> leds.setLEDstate("yellow")),
//                    new RobotAlignToTagRange(drivetrain, webcam, "back", 4, targetBackdropTag.id, 3, true),
                    new RelocalizeFromTagCommand(drive, aprilTagProcessor),
                    new ParallelCommandGroup(
                            new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "deposit"),
                            new SequentialCommandGroup(
                                    new WaitCommand(250),
                                    new LiftToPositionCommand(lift, 200, 10)
                            )
                    ),
                    new InstantCommand(gripper::releaseRight),
                    new WaitCommand(100),
                    new InstantCommand(gripper::releaseLeft),
                    new WaitCommand(100),
                    new FollowTrajectoryCommand(drive,
                            drive.trajectorySequenceBuilder(SpikeToBackdrop.end())
                                    .back(2, velConstraint10in, accelConstraint40in)
                                    .build()
                    ),
                    new InstantCommand(() -> leds.setLEDstate("purple")),
                    new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "intake")
//                new AutoBackdropDepositCommand(arm, wrist, gripper, lift, intake, winch, leds, drivetrain, webcam, drive, targetBackdropTag)
            ));
        else // don't deliver yellow
//            if(cycle)
//                //tbd
//                else


            if (cycle) {
                autoCommands.addCommands(new SequentialCommandGroup(
                        // back to intake to back AutoBackToStackToBackCommand
                        new InstantCommand(() -> leds.setLEDstate("plaid")),
                        new FollowTrajectoryCommand(drive, BackToStack),
                        // intake in sequence
                        new ParallelCommandGroup(
                                new IntakeInCommand(intake, leds),
                                new FollowTrajectoryCommand(drive, StackPickupSequence)
                        ),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> leds.setLEDstate("yellow")),
                                new SequentialCommandGroup(
                                        new InstantCommand(gripper::grabLeft),
                                        new InstantCommand(gripper::grabRight),
                                        new WaitCommand(200),
                                        new IntakeOutCommand(intake)
                                ),
                                new FollowTrajectoryCommand(drive, StackToBack)
                        )

                ));

                // deposit dequence
                if (cycleTarget == "backdrop") {
                    autoCommands.addCommands(new SequentialCommandGroup(
                            new AutoBackdropDepositCommand(arm, wrist, gripper, lift, intake, winch, leds, drivetrain, webcam, drive, targetBackdropTag)
                    ));
                } else {
                    autoCommands.addCommands(new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> leds.setLEDstate("white")),
                                    new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "dropPurple")
                            ),
                            new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(drive.getPoseEstimate()).setReversed(true).forward(2, velConstraint10in, accelConstraint40in).build()),
                            new InstantCommand(gripper::releaseRight),
                            new InstantCommand(gripper::releaseLeft),
                            new WaitCommand(100),
                            new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(drive.getPoseEstimate()).setReversed(true).back(2, velConstraint10in, accelConstraint40in).build()),
                            new InstantCommand(() -> leds.setLEDstate("purple")),
                            new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "intake")
                    ));
                }
                // cycle code
                // park in middle code
            } else {
                //park code
//                if (parkIn == "corner") {
//                    autoCommands.addCommands(new SequentialCommandGroup(
//                            new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(drive.getPoseEstimate()).setReversed(true).strafeRight(24, velConstraint20in, accelConstraint40in).build()),
//                            new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(drive.getPoseEstimate()).setReversed(true).back(24, velConstraint20in, accelConstraint40in).build())
//                    ));
//                } else {
//                    autoCommands.addCommands(new SequentialCommandGroup(
//                            new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(drive.getPoseEstimate()).setReversed(true).strafeLeft(24, velConstraint20in, accelConstraint40in).build()),
//                            new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(drive.getPoseEstimate()).setReversed(true).back(24, velConstraint20in, accelConstraint40in).build())
//                    ));
//                }
            }

        return autoCommands;
    }
}
