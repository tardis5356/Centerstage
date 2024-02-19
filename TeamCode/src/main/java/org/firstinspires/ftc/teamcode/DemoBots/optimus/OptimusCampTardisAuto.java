package org.firstinspires.ftc.teamcode.DemoBots.optimus;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DemoBots.optimus.Optimus_subsystems.OptimusDrive;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.commands.OptimusAdjustLeftCommand;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.commands.OptimusAdjustRightCommand;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.commands.OptimusDriveBackwardsCommand;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.commands.OptimusDriveForwardCommand;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.commands.OptimusStopDrivingCommand;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.commands.OptimusTurnIMUCommand;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.commands.OptimusTurnLeftCommand;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.commands.OptimusTurnRightCommand;

@Autonomous(name = "OptimusAuto")
public class OptimusCampTardisAuto extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private OptimusDrive drivetrain;

    @Override
    public void initialize() {
        drivetrain = new OptimusDrive(hardwareMap);

        waitForStart();

        schedule(
                new SequentialCommandGroup(
                        new OptimusDriveForwardCommand(drivetrain),
                        new WaitCommand(2000),
                        new OptimusStopDrivingCommand(drivetrain)
//
//                        new OptimusTurnIMUCommand(drivetrain, 90, 2),
//
//                        new OptimusDriveForwardCommand(drivetrain),
//                        new WaitCommand(1000),
//
//                        new OptimusTurnIMUCommand(drivetrain, 180, 2),
//
//                        new OptimusDriveForwardCommand(drivetrain),
//                        new WaitCommand(1000),
//
//                        new OptimusTurnIMUCommand(drivetrain, 270, 2),
//
//                        new OptimusDriveForwardCommand(drivetrain),
//                        new WaitCommand(1000),
//
//                        new OptimusTurnIMUCommand(drivetrain, 0, 2)
                ));
    }
}