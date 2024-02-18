package org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos;


import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.BlueBackstage_StartToOrigin;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.BlueWings_StartToOrigin;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedBackstage_StartToOrigin;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.RedWings_StartToOrigin;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.blueBackstage_StartPos;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.blueWings_StartPos;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.redBackstage_StartPos;
import static org.firstinspires.ftc.teamcode.ARTEMIS.auto.EXCCMP_Autos.EXCCMP_AutoTrajectories.redWings_StartPos;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ARTEMIS.auto.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.ARTEMIS.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ARTEMIS.trajectorysequence.TrajectorySequence;

//@Disabled
@Autonomous(group = "drive", name = "StartPoseContinuityTest")
public class StartPoseContinuityTest extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private SampleMecanumDrive drive;

    private Gamepad currentGamepad, previousGamepad;

    private int path = 0;

    private TrajectorySequence testTraj;

    @Override
    public void initialize() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);
        drive = new SampleMecanumDrive(hardwareMap);

        EXCCMP_AutoTrajectories.generateTrajectories(drive);

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Press DPAD UP to cycle starting position");
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up)
                path++;

            if(path == 0){
                testTraj = RedWings_StartToOrigin;
                drive.setPoseEstimate(redWings_StartPos);
                telemetry.addData("Starting: RedWings", redWings_StartPos);
            } else if(path == 1){
                testTraj = RedBackstage_StartToOrigin;
                drive.setPoseEstimate(redBackstage_StartPos);
                telemetry.addData("Starting: RedBackstage", redBackstage_StartPos);
            } else if(path == 2){
                testTraj = BlueWings_StartToOrigin;
                drive.setPoseEstimate(blueWings_StartPos);
                telemetry.addData("Starting: BlueWings", blueWings_StartPos);
            } else if(path == 3){
                testTraj = BlueBackstage_StartToOrigin;
                drive.setPoseEstimate(blueBackstage_StartPos);
                telemetry.addData("Starting: BlueBackstage", blueBackstage_StartPos);
            } else {
                path = 0;
            }

            telemetry.addLine("waitForStart");
            telemetry.update();
            sleep(20);
        }

        telemetry.update();
        // drop purple
        schedule(new SequentialCommandGroup(
           new FollowTrajectoryCommand(drive, testTraj)
        ));
    }
}