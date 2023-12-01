package org.firstinspires.ftc.teamcode.TESTBED.auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.TESTBED.trajectorysequence.TrajectorySequence;

public class CSTB_FollowTrajectoryCommand extends CommandBase {
    private CSTB_SampleMecanumDrive drive;
    private TrajectorySequence sequence;

    public CSTB_FollowTrajectoryCommand(CSTB_SampleMecanumDrive drive, TrajectorySequence sequence){
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

