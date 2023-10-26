package org.firstinspires.ftc.teamcode.DemoBots.primus;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.DemoBots.primus.commands.BotToIntake;
import org.firstinspires.ftc.teamcode.DemoBots.primus.Primus_subsystems.Gripper;

public class primus_command_based extends CommandOpMode{
    private GamepadEx driver;
    private Gripper left;
    private Gripper right;
    private BotToIntake botToIntake;

    DcMotor BackRight;



    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        left = new Gripper(hardwareMap);
        right = new Gripper(hardwareMap);
        BackRight = hardwareMap.get(DcMotor.class, "mBR");

        botToIntake = new BotToIntake(left, right);
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .2)
                .whenActive(botToIntake);
    }

    @Override
    public void run(){
        super.run();

        telemetry.addData("ArmPosition", BackRight.getCurrentPosition());
        telemetry.update();
    }
}
