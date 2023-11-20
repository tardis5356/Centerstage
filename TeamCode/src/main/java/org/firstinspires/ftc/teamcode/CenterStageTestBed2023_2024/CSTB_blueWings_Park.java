package org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024;

import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.blueWings_ToDecisionPoint;
import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.blueWings_ToMiddlePark;
import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.redWings_ToMiddlePark;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group = "drive", name = "CSTB blue wings PARK")
public class CSTB_blueWings_Park extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private CSTB_SampleMecanumDrive drive;
    //    private Lift lift;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize() {
//        MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new CSTB_SampleMecanumDrive(hardwareMap);


        drive.setPoseEstimate(CSTB_AutoTrajectories.blueWings_StartPos);
        CSTB_AutoTrajectories.generateTrajectories(drive);

        //gripper.close();

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {

            telemetry.addLine("waitForStart");
            telemetry.update();
            sleep(20);
        }


        telemetry.update();
        schedule(new SequentialCommandGroup(

                new CSTB_FollowTrajectoryCommand(drive, blueWings_ToDecisionPoint),
        new CSTB_FollowTrajectoryCommand(drive, blueWings_ToMiddlePark)
        ));

    }
}

/*

package org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024;

import static org.firstinspires.ftc.teamcode.CenterStageTestBed2023_2024.CSTB_AutoTrajectories.blueWings_StartToBackstage;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CSTB_blueWings_Park {

    @Autonomous(group = "drive", name = "CSTB blue wings PARK")
    public class CSTB_redBackstage_Auto extends LinearOpMode {
        ElapsedTime runtime = new ElapsedTime();

        private CSTB_SampleMecanumDrive drive;
        //    private Lift lift;

        FtcDashboard dashboard = FtcDashboard.getInstance();

        @Override
        public void runOpMode() {
            drive = new CSTB_SampleMecanumDrive(hardwareMap);

            drive.setPoseEstimate(CSTB_AutoTrajectories.blueWings_StartPos);
            CSTB_AutoTrajectories.generateTrajectories(drive);

            CommandScheduler.getInstance().schedule(//new SequentialCommandGroup(
                    new CSTB_FollowTrajectoryCommand(drive, blueWings_StartToBackstage)
            );

            //gripper.close();

            telemetry.setMsTransmissionInterval(50);

            while (!isStarted() && !isStopRequested()) {
                telemetry.addLine("waitForStart");
                telemetry.update();
                sleep(20);
            }

            while (!isStopRequested() && opModeIsActive()) {
                CommandScheduler.getInstance().run();
                telemetry.update();
            }
        }
    }
}

 */