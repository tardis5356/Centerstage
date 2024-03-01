package org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos;

import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_Auto.park;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ARTEMIS.auto.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.LiftToPositionCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.RobotToStateCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.autoCommands.RelocalizeFromTagCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Arm;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions;
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
    /*

    TrajectorySequence StartToSpike, // both
    TrajectorySequence SpikeToStack, // wing
    TrajectorySequence SpikeToBackdropYellow, // backstage
    TrajectorySequence StackToStackWaypoint, // wing, cycle
    TrajectorySequence StackWaypointToBackWaypoint, // wing, cycle
    TrajectorySequence BackWaypointToBackdropYellow, // wing, cycle
    TrajectorySequence BackWaypointToBackdropWhite, // wing, cycle
    TrajectorySequence BackdropRelocWaypointToBackdrop, //backstage
    TrajectorySequence BackWaypointToBackstage, // wing, cycle, park
    TrajectorySequence BackdropYellowSlotToWhiteSlot, // wing, cycle
    TrajectorySequence BackdropToPark, // both
    TrajectorySequence BackdropToBackdropWaypoint, // cycle
    TrajectorySequence BackdropWaypointToStackWaypoint, // cycle
    TrajectorySequence StackWaypointToStack, // cycle

     */
    private SequentialCommandGroup autoCommands = new SequentialCommandGroup();


    public SequentialCommandGroup generateAutoCommands2(Arm arm, Wrist wrist, Gripper gripper, Lift lift, Intake intake, Winch winch, LEDs leds, Drivetrain drivetrain, Webcams webcam, AprilTagMetadata targetBackdropTag, AprilTagProcessor aprilTagProcessor, SampleMecanumDrive drive,
                                                        TrajectorySequence StartToSpike, // both
                                                        TrajectorySequence SpikeToStack, // wing
                                                        TrajectorySequence SpikeToBackdropYellow, // backstage
                                                        TrajectorySequence StackToStackWaypoint, // wing, cycle
                                                        TrajectorySequence StackWaypointToBackWaypoint, // wing, cycle
                                                        TrajectorySequence BackWaypointToBackdropYellow, // wing, cycle
                                                        TrajectorySequence BackWaypointToBackdropWhite, // cycle
                                                        TrajectorySequence BackdropRelocWaypointToBackdrop, //backstage
                                                        TrajectorySequence BackWaypointToBackstage, // cycle, park
                                                        TrajectorySequence BackdropYellowSlotToWhiteSlot, // wing, cycle
                                                        TrajectorySequence BackdropToPark, // both
                                                        TrajectorySequence BackdropToBackdropWaypoint, // cycle
                                                        TrajectorySequence BackdropWaypointToStackWaypoint, // cycle
                                                        TrajectorySequence StackWaypointToStack, // cycle
                                                        String alliance, String startingSide, String cycleTarget, String transitVia, String parkIn,
                                                        boolean cycle, boolean wait, boolean deliverYellow, Telemetry telemetry) {

        /**
         * general stack pickup sequence
         * */
        SequentialCommandGroup stackPickup = new SequentialCommandGroup(
                new InstantCommand(intake::in),
                new InstantCommand(() -> leds.setLEDstate("intaking")),
                new InstantCommand(intake::downThirdPixel),
                new WaitCommand(300),
//                new InstantCommand(intake::downFifthPixel),
//                new WaitCommand(250),
                new InstantCommand(intake::out),
                new WaitCommand(300),
                new InstantCommand(intake::in),
                new ParallelCommandGroup(
                        new FollowTrajectoryCommand(drive, StackToStackWaypoint),
                        new SequentialCommandGroup(
//                                new WaitCommand(200),
//                                new InstantCommand(intake::in),
                                new WaitCommand(300),
                                new InstantCommand(intake::up),
                                new WaitCommand(500),
                                new InstantCommand(intake::out)
                        )
                )
        );

        SequentialCommandGroup stackPickup2 = new SequentialCommandGroup(
                new InstantCommand(intake::in),
                new InstantCommand(() -> leds.setLEDstate("intaking")),
                new InstantCommand(intake::downThirdPixel),
                new WaitCommand(300),
//                new InstantCommand(intake::downFifthPixel),
//                new WaitCommand(250),
                new InstantCommand(intake::out),
                new WaitCommand(300),
                new InstantCommand(intake::in),
                new ParallelCommandGroup(
                        new FollowTrajectoryCommand(drive, StackToStackWaypoint),
                        new SequentialCommandGroup(
//                                new WaitCommand(200),
//                                new InstantCommand(intake::in),
                                new WaitCommand(300),
                                new InstantCommand(intake::up),
                                new WaitCommand(500),
                                new InstantCommand(intake::out)
                        )
                )
        );

        SequentialCommandGroup relocSequence = new SequentialCommandGroup(
                new InstantCommand(() -> leds.setLEDstate("red_scan")),
                new WaitCommand(200),
                new RelocalizeFromTagCommand(drive, drivetrain, aprilTagProcessor, telemetry),
                new WaitCommand(50),
                new InstantCommand(() -> leds.setLEDstate("green"))
        );

        SequentialCommandGroup relocSequence2 = new SequentialCommandGroup(
                new InstantCommand(() -> leds.setLEDstate("red_scan")),
                new WaitCommand(200),
                new RelocalizeFromTagCommand(drive, drivetrain, aprilTagProcessor, telemetry),
                new WaitCommand(50),
                new InstantCommand(() -> leds.setLEDstate("green"))
        );

        /***
         * START OF AUTO
         */

        /**
         * deliver purple pixel
         */
        autoCommands.addCommands(new SequentialCommandGroup(
                new InstantCommand(() -> leds.setLEDstate("green")),
                new ParallelCommandGroup(
                        new FollowTrajectoryCommand(drive, StartToSpike),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "dropPurple")
                        )
                ),
                new WaitCommand(50),
//                new InstantCommand(gripper::releaseLeft),
                new InstantCommand(gripper::purpleReleaseAuto),
                new WaitCommand(200)
        ));

        // get extra pixel on first cycle (start: spike, end: back)
        if (startingSide == "wing") {
            /**
             * get extra pixel on first cycle if wing
             * start: Spike
             * end: BackWaypoint
             */
            autoCommands.addCommands(new SequentialCommandGroup(
                    new InstantCommand(() -> leds.setLEDstate("white")),
                    new ParallelCommandGroup(
                            new FollowTrajectoryCommand(drive, SpikeToStack),
                            new ParallelDeadlineGroup(
                                    new WaitCommand(2000), //TODO: fix this bad command isFinished
                                    new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds,
                                            "intakeNoRelease")
                            )
                    ),
                    new WaitCommand(100),

                    // INTAKE SEQUENCE
                    stackPickup, //ends at stack waypoint

                    new ParallelCommandGroup(
                            new InstantCommand(() -> leds.setLEDstate("yellow")),
//                            new SequentialCommandGroup(
                            new InstantCommand(gripper::grabLeft),
                            new InstantCommand(gripper::grabRight),
                            new InstantCommand(arm::toTransition),
                            new InstantCommand(wrist::toTransition),
//                                    new InstantCommand(intake::up),
//                                    new WaitCommand(150),
//                                    new InstantCommand(intake::out)
//                            ),
                            new FollowTrajectoryCommand(drive, StackWaypointToBackWaypoint)
                    )
            ));
        }

        // align to atag and deposit sequence
        if (deliverYellow) {
            if (startingSide == "wing") {
                /**
                 * waypoint to backdrop (when coming from wing)
                 * start: backdrop waypoint
                 * end: backdrop yellow pos
                 */
                autoCommands.addCommands(new SequentialCommandGroup(
                        relocSequence,
                        new ParallelCommandGroup(
//                                new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "deposit"),
                                new InstantCommand(arm::toDeposit),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new InstantCommand(wrist::tiltToDeposit),
                                        new InstantCommand(wrist::rollToPurpleAuto),
                                        new LiftToPositionCommand(lift, 250, 10)
                                ),
                                new SequentialCommandGroup(
                                        new WaitCommand(250),
                                        new FollowTrajectoryCommand(drive, BackWaypointToBackdropYellow)
                                )
                        )
                ));
            } else {
                /**
                 * go to backdrop if backstage
                 * start: spike
                 * end: backdrop yellow pos
                 */
                autoCommands.addCommands(
                        new SequentialCommandGroup( // turn bot around to align with backdrop
                                new InstantCommand(arm::toDeposit),
                                new InstantCommand(wrist::tiltToDeposit),
//                                new InstantCommand(() -> wrist.setRollIndex(90)),
                                new InstantCommand(() -> wrist.rollToPurpleAuto()),
                                new LiftToPositionCommand(lift, 250, 10),
                                new FollowTrajectoryCommand(drive, SpikeToBackdropYellow),
                                relocSequence,
                                new FollowTrajectoryCommand(drive, BackdropRelocWaypointToBackdrop)
//                                new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "deposit"),
//                                new InstantCommand(arm::toDeposit),
//                                new SequentialCommandGroup(
//                                new WaitCommand(50),
//                                new InstantCommand(wrist::tiltToDeposit),
//                                )
                        ));
            }
            /**
             * deliver yellow pixel
             * start: yellow slot
             * end: yellow slot
             */
            autoCommands.addCommands(new SequentialCommandGroup(
//                    new InstantCommand(gripper::releaseRight), // drop yellow
                    new InstantCommand(gripper::yellowReleaseAuto), // drop yellow
                    new WaitCommand(150)
//                    new InstantCommand(() -> { /** MOVE BOT TO HIGH DELVIERY POSITION */
//                        BotPositions.ARM_HIGH_POSITION = true;
//                        arm.toDeposit();
//                        wrist.tiltToDeposit();
//                        BotPositions.ARM_HIGH_POSITION = false;
//                    })
            ));

            /**
             * move to white delivery
             * start: yellow slot
             * end: white slot
             */
            if (startingSide == "wing") {
                autoCommands.addCommands(new SequentialCommandGroup(
                        new InstantCommand(() -> leds.setLEDstate("white")),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new InstantCommand(() -> { /** MOVE BOT TO LOW DELVIERY POSITION */
                                            BotPositions.ARM_HIGH_POSITION = false;
                                            arm.toDeposit();
                                            wrist.tiltToDeposit();
                                        })
                                ),
                                new FollowTrajectoryCommand(drive, BackdropYellowSlotToWhiteSlot)
                        ),
                        new InstantCommand(gripper::purpleReleaseAuto),
                        new WaitCommand(100)
                ));
            }
        } else {
            // don't deliver yellow
        }

        if (cycle) {
            /**
             * stack pickup sequence
             * start: backdrop
             * end: stack waypoint
             */
            autoCommands.addCommands(new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new ParallelDeadlineGroup(
                                    new WaitCommand(2000),
                                    new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "intake")
                            ),
                            new FollowTrajectoryCommand(drive, BackdropToBackdropWaypoint)
                    ),
                    new InstantCommand(() -> leds.setLEDstate("plaid")),
                    new FollowTrajectoryCommand(drive, BackdropWaypointToStackWaypoint),
                    new InstantCommand(() -> leds.setLEDstate("yellow")),
                    new FollowTrajectoryCommand(drive, StackWaypointToStack),
                    stackPickup
            ));

            if (cycleTarget == "backdrop") {
                /**
                 * cycle deposit sequence
                 * start: stack waypoint
                 * end: backdrop
                 */
                autoCommands.addCommands(new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> leds.setLEDstate("plaid")),
                                new SequentialCommandGroup(
                                        new InstantCommand(gripper::grabLeft),
                                        new InstantCommand(gripper::grabRight),
                                        new InstantCommand(intake::up),
                                        new WaitCommand(150),
//                                    new IntakeOutCommand(intake)
                                        new InstantCommand(intake::out),
                                        new WaitCommand(200),
                                        new InstantCommand(arm::toTransition),
                                        new InstantCommand(wrist::toTransition)
                                ),
                                new FollowTrajectoryCommand(drive, StackWaypointToBackWaypoint)
                        ),
                        relocSequence2,
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(arm::toDeposit),
                                        new WaitCommand(500),
                                        new InstantCommand(wrist::tiltToDeposit)
                                ),
//                                new InstantCommand(() -> wrist.setRollIndex(90)),
                                new InstantCommand(() -> wrist.rollToCentered()),
                                new LiftToPositionCommand(lift, 250, 10),
                                new FollowTrajectoryCommand(drive, BackWaypointToBackdropWhite)
                        ),
                        new InstantCommand(gripper::releaseLeft),
                        new InstantCommand(gripper::releaseRight)
                ));
            } else {
                autoCommands.addCommands(new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> leds.setLEDstate("plaid")),
                                new SequentialCommandGroup(
                                        new InstantCommand(gripper::grabLeft),
                                        new InstantCommand(gripper::grabRight),
                                        new InstantCommand(intake::up),
                                        new WaitCommand(150),
//                                    new IntakeOutCommand(intake)
                                        new InstantCommand(intake::out),
                                        new WaitCommand(200),
                                        new InstantCommand(arm::toTransition),
                                        new InstantCommand(wrist::toTransition)
                                )//,
//                                new FollowTrajectoryCommand(drive, StackWaypointToBackWaypoint)
                        ),
//                        relocSequence,
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive, BackWaypointToBackstage),
                                new SequentialCommandGroup(
                                        new WaitCommand(1500),
                                        new ParallelDeadlineGroup(
                                                new WaitCommand(2000),
                                                new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "dropPurple")
                                        )
                                )
                        ),
                        new InstantCommand(gripper::releaseLeft),
                        new InstantCommand(gripper::releaseRight)
                ));
            }

        }

        if (park) {
            //park code
            autoCommands.addCommands(new ParallelCommandGroup(
                    new SequentialCommandGroup(
                            new WaitCommand(300),
                            new ParallelDeadlineGroup(
                                    new WaitCommand(1200),
                                    new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "intake")
                            )
                    ),
                    new FollowTrajectoryCommand(drive, BackdropToPark)
            ));
            //}
        } else {
            autoCommands.addCommands(new SequentialCommandGroup(
                    new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "intake")
            ));
        }
        return autoCommands;
    }
}
