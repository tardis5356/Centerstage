package org.firstinspires.ftc.teamcode.ARTEMIS.commands.autoCommands;

import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.accelConstraint40in;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.velConstraint10in;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ARTEMIS.auto.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.LiftToPositionCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.RobotAlignToTagRange;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.RobotToStateCommand;
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
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

public class AutoBackdropDepositCommand extends SequentialCommandGroup {
    public AutoBackdropDepositCommand(Arm arm, Wrist wrist, Gripper gripper, Lift lift, Intake intake, Winch winch, LEDs leds, Drivetrain drivetrain, Webcams webcam, SampleMecanumDrive drive, AprilTagMetadata targetBackdropTag){
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(() -> leds.setLEDstate("yellow")),
                        new RobotAlignToTagRange(drivetrain, webcam, "back", 4, targetBackdropTag.id, 3, true),
                        new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "deposit"),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new LiftToPositionCommand(lift, 50, 10)
                        )
                ),
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(drive.getPoseEstimate()).setReversed(true).forward(2, velConstraint10in, accelConstraint40in).build()),
                new InstantCommand(gripper::releaseRight),
                new WaitCommand(100),
                new InstantCommand(gripper::releaseLeft),
                new WaitCommand(100),
                new FollowTrajectoryCommand(drive, drive.trajectorySequenceBuilder(drive.getPoseEstimate()).setReversed(true).back(2, velConstraint10in, accelConstraint40in).build()),
                new InstantCommand(() -> leds.setLEDstate("purple")),
                new RobotToStateCommand(arm, wrist, gripper, lift, intake, winch, leds, "intake")
        );
    }
}
