package org.firstinspires.ftc.teamcode.DemoBots.optimus;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.Optimus_subsystems.OptimusDrive;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.commands.AdjustLeftCommand;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.commands.AdjustRightCommand;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.commands.DriveBackwardsCommand;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.commands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.commands.StopDrivingCommand;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.commands.TurnLeftCommand;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.commands.TurnRightCommand;
import org.firstinspires.ftc.teamcode.TESTBED.auto.CSTB_AutoTrajectories;
import org.firstinspires.ftc.teamcode.TESTBED.auto.CSTB_FollowTrajectoryCommand;


@Autonomous(name = "OptimusAuto")
public class OptimusCampTardisAuto extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;
    private OptimusDrive drivetrain;
    private DriveForwardCommand goForward;
    private DriveBackwardsCommand goBackwards;
    private TurnLeftCommand turnLeft;
    private TurnRightCommand turnRight;
    private AdjustLeftCommand aLittleLeft;
    private AdjustRightCommand aLittleRight;
    private StopDrivingCommand stop;
    boolean TurnL;
    boolean TurnR;




    @Override
    public void initialize() {
        drivetrain = new OptimusDrive(hardwareMap);
        goForward = new DriveForwardCommand(drivetrain);
        goBackwards = new DriveBackwardsCommand(drivetrain);
        turnLeft = new TurnLeftCommand(drivetrain);
        turnRight = new TurnRightCommand(drivetrain);
        aLittleLeft = new AdjustLeftCommand(drivetrain);
        aLittleRight = new AdjustRightCommand(drivetrain);
        stop = new StopDrivingCommand(drivetrain);

        imu = hardwareMap.get(BNO055IMU.class, "adafruitIMU");

        imu.initialize(new BNO055IMU.Parameters());

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

            Orientation orientation = imu.getAngularOrientation();

//        double CurrentAngle = orientation.firstAngle;
//
//            if(CurrentAngle == Math.toRadians(45)){
//                TurnL = true;
//                TurnR = false;
//            }
//            if(CurrentAngle == Math.toRadians(315)){
//                TurnR = true;
//                TurnL = false;
//            }

            waitForStart();

            if(runtime.seconds() <= 30) {
                schedule(new SequentialCommandGroup(
                        goForward,
                        new WaitCommand(1000),
                        turnLeft,
                        new WaitCommand(500),
                        goForward,
                        new WaitCommand(1000),
                        turnRight,
                        new WaitCommand(500),
                        goForward,
                        new WaitCommand(1000),
                        stop

                ));

            }


//            while (opModeIsActive()){
//                telemetry.update();
//                telemetry.addData("orientation", CurrentAngle);
//            }


    }

}
