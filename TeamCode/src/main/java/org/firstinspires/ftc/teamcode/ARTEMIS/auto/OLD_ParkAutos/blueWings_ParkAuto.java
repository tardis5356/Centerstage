package org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_ParkAutos;

import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_ParkAutos.Artemis_ParkAutoTrajectories.blueWings_DecisionPointToCenterPark;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.OLD_ParkAutos.Artemis_ParkAutoTrajectories.blueWings_StartPositionToDecisionPoint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.ARTEMIS.auto.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;

@Disabled
@Autonomous(group = "drive", name = "A BlueWings PARK")
public class blueWings_ParkAuto extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private SampleMecanumDrive drive;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize() {
//        MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(Artemis_ParkAutoTrajectories.blueWings_StartPos);
        Artemis_ParkAutoTrajectories.generateTrajectories(drive);

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {

            telemetry.addLine("waitForStart");
            telemetry.update();
            sleep(20);
        }


        telemetry.update();
        schedule(new SequentialCommandGroup(

                new FollowTrajectoryCommand(drive, blueWings_StartPositionToDecisionPoint),
                new FollowTrajectoryCommand(drive, blueWings_DecisionPointToCenterPark)

        ));

    }
}