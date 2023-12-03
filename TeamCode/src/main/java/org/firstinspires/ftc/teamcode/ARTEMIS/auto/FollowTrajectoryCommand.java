package org.firstinspires.ftc.teamcode.ARTEMIS.auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ARTEMIS.trajectorysequence.TrajectorySequence;

public class FollowTrajectoryCommand extends CommandBase {
    private SampleMecanumDrive drive;
    private TrajectorySequence sequence;

    public FollowTrajectoryCommand(SampleMecanumDrive drive, TrajectorySequence sequence){
        this.drive = drive;
        this.sequence = sequence;
    }

    @Override
    public void initialize(){
        drive.followTrajectorySequenceAsync(sequence);
    }

    @Override
    public void execute(){
        drive.update();
    }

    @Override
    public boolean isFinished(){
        return !drive.isBusy();
    }

}

