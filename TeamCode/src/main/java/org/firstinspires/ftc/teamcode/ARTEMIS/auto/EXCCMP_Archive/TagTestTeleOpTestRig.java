package org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Archive;

import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.AutoUtils.relocalize6;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.AutoUtils.relocalizeFrontCamera;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.RobotAlignToTagRange;
import org.firstinspires.ftc.teamcode.ARTEMIS.commands.RobotToStateCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Arm;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Intake;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.LEDs;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Lift;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Webcams;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Winch;
import org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.Wrist;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

//@Disabled
@TeleOp(name = "TagTestTeleopTestRig", group = "AA")
public class TagTestTeleOpTestRig extends CommandOpMode {
    //gamepads
    private GamepadEx driver1, driver2;

    //drivetrain motors and variables
    double FB, LR, Rotation;

    //keep track of time to ensure that e-game specific items dont trigger
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime rumbleTimer = new ElapsedTime();

    // speed multiplier (granny mode)
    double FAST_SPEED_MULTIPLIER = 1;
    double SLOW_SPEED_MULTIPLIER = 0.5;
    double CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER;

    public enum RobotState {
        INTAKE,
        INTAKING,
        TRANSITION,
        DEPOSIT
    }

    org.firstinspires.ftc.teamcode.ARTEMIS.teleop.Gen1_TeleOp.RobotState robotState = org.firstinspires.ftc.teamcode.ARTEMIS.teleop.Gen1_TeleOp.RobotState.INTAKE;

    public static boolean intakeStageActive = true;
    public static boolean highArmPosition = false;

    private Drivetrain drivetrain;

    private Webcams webcams;

    IMU imu;

    @Override
    public void initialize() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        //init controllers
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);


        webcams = new Webcams(hardwareMap);


        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )
                )
        );

        imu.resetYaw();

        webcams.setCamera("front");
        webcams.setActiveProcessor("apriltag");
    }

    @Override
    // This is pretty much your while OpMode is active loop
    public void run() {
        super.run();

        double start = System.nanoTime();

        double currentThetaRads = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        telemetry.addData("# of detections: ", webcams.getActiveAprilTagProcessor().getDetections().size());

        telemetry.addLine();

        telemetry.addData("localize6", relocalizeFrontCamera(webcams.getCurrentDetections(webcams.getActiveAprilTagProcessor()), currentThetaRads, telemetry));

        telemetry.update();
    }

}