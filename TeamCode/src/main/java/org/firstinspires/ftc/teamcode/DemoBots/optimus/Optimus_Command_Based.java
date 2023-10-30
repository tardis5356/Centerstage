package org.firstinspires.ftc.teamcode.DemoBots.optimus;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.DemoBots.optimus.commands.RobotToIntakePositionCommand;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.Optimus_subsystems.Gripper;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.Optimus_subsystems.Wrist;

public class Optimus_Command_Based extends CommandOpMode {
    private GamepadEx driver;

    private Wrist wrist;
    private Gripper gripper;

    private RobotToIntakePositionCommand robotToIntakePositionCommand;

    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);

        gripper = new Gripper(hardwareMap);
        wrist = new Wrist(hardwareMap);

        robotToIntakePositionCommand = new RobotToIntakePositionCommand(gripper, wrist);

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(robotToIntakePositionCommand);
    }

    @Override
    public void run() {
        super.run();

    }
}
