package org.firstinspires.ftc.teamcode.DemoBots.optimus.Optimus_subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions;

public class OptimusDrive extends SubsystemBase {
    private DcMotor LeftDrive;
    private DcMotor RightDrive;
    public OptimusDrive(HardwareMap hardwareMap){
        LeftDrive = hardwareMap.get(DcMotor.class, "mL");
        RightDrive = hardwareMap.get(DcMotor.class, "mR");
    }
    @Override

    // periodic is a method that runs in the background. pretty useful if you have other methods that you want to trigger automatically.
    // it is needed even if it doesn't have anything.
    public void periodic() {
    }
    public void DriveForward() {
        LeftDrive.setPower(-.75);
        RightDrive.setPower(.75);
    }
    public void DriveBackward(){
        LeftDrive.setPower(.75);
        RightDrive.setPower(-.75);
    }
    public void TurnLeft(){
        LeftDrive.setPower(.5);
        RightDrive.setPower(.5);
    }
    public void TurnRight(){
        LeftDrive.setPower(-0.5);
        RightDrive.setPower(-0.5);
    }
    public void adjustLeft(){
        LeftDrive.setPower(-.75);
        RightDrive.setPower(.78);
    }
    public void adjustRight(){
        LeftDrive.setPower(-.78);
        RightDrive.setPower(.75);
    }
    public void stopDriving(){
        LeftDrive.setPower(0);
        RightDrive.setPower(0);
    }
}
