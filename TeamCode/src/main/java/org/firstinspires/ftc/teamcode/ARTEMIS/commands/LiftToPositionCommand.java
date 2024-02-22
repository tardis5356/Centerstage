package org.firstinspires.ftc.teamcode.ARTEMIS.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Lift;


public class LiftToPositionCommand extends CommandBase {
    private Lift lift;

    int targetPosition;
    int tolerance;

    public LiftToPositionCommand(Lift lift, int targetPosition, int tolerance) {
        this.lift = lift;
        this.targetPosition = targetPosition;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() { // runs once
//        lift.setTargetPosition(targetPosition);
    }

    @Override
    public void execute() { // runs continuously
        //  lift.setTolerance(tolerance);
        lift.setTargetPosition(targetPosition);
        //  lift.updatePIDValues();
    }

    @Override
    public boolean isFinished() { // returns true when finished
        if (targetPosition == -10)
            return true;
        else
            return Math.abs(lift.getLiftPosition() - targetPosition) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
//        lift.stop();
    }

}