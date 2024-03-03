package org.firstinspires.ftc.teamcode.DemoBots.primus;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.DemoBots.primus.Primus_subsystems.PrimusDrive;
import org.firstinspires.ftc.teamcode.DemoBots.primus.commands.PrimusDriveForwardCommand;
import org.firstinspires.ftc.teamcode.DemoBots.primus.commands.PrimusTurnIMUCommand;

@Disabled
@Autonomous(name = "PrimusAuto", group="demo")
public class PrimusCampTARDISAuto extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private PrimusDrive drivetrain;

    @Override
    public void initialize() {
        drivetrain = new PrimusDrive(hardwareMap);

        waitForStart();

        schedule(new SequentialCommandGroup(
                new PrimusDriveForwardCommand(drivetrain),
                new WaitCommand(300),

                new PrimusTurnIMUCommand(drivetrain, 90, 2),

                new PrimusDriveForwardCommand(drivetrain),
                new WaitCommand(300),

                new PrimusTurnIMUCommand(drivetrain, 180, 2),

                new PrimusDriveForwardCommand(drivetrain),
                new WaitCommand(300),

                new PrimusTurnIMUCommand(drivetrain, 270, 2),

                new PrimusDriveForwardCommand(drivetrain),
                new WaitCommand(300),

                new PrimusTurnIMUCommand(drivetrain, 0, 2)
        ));
    }
}