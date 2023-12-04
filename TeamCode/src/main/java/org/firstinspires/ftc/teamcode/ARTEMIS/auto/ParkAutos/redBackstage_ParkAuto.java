package org.firstinspires.ftc.teamcode.ARTEMIS.auto.ParkAutos;

import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.ParkAutos.Artemis_ParkAutoTrajectories.redBackstage_StartPositionToCornerPark;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.ARTEMIS.auto.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TESTBED.auto.CSTB_AutoTrajectories;

@Autonomous(group = "drive", name = "A RedBackstage PARK")
public class redBackstage_ParkAuto extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private SampleMecanumDrive drive;

    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void initialize() {
//        MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(Artemis_ParkAutoTrajectories.redBackstage_StartPos);
        Artemis_ParkAutoTrajectories.generateTrajectories(drive);

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {

            telemetry.addLine("waitForStart");
            telemetry.update();
            sleep(20);
        }


        telemetry.update();
        schedule(new SequentialCommandGroup(

                new FollowTrajectoryCommand(drive, redBackstage_StartPositionToCornerPark)

        ));

    }
}