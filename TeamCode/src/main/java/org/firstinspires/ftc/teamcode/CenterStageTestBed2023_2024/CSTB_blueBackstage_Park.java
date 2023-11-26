package org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024;

import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.blueBackstage_ToDecisionPoint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

//public class CSTB_blueBackstage_Park {

    @Autonomous(group = "drive", name = "CSTB blue backstage PARK")
public class CSTB_blueBackstage_Park extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private CSTB_SampleMecanumDrive drive;
    //    private Lift lift;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize() {
//        MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new CSTB_SampleMecanumDrive(hardwareMap);


        drive.setPoseEstimate(CSTB_AutoTrajectories.blueBackstage_StartPos);
        CSTB_AutoTrajectories.generateTrajectories(drive);

        //gripper.close();

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {

            telemetry.addLine("waitForStart");
            telemetry.update();
            sleep(20);

            telemetry.update();
            schedule(new SequentialCommandGroup(

                    new CSTB_FollowTrajectoryCommand(drive, blueBackstage_ToDecisionPoint)
            ));

        }
    }
    }
