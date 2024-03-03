package org.firstinspires.ftc.teamcode.TESTBED.auto;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name="CSTB_Teleop")
public class
CSTB_Teleop extends Base_CSTB{

    static final double POWER_MULTIPLIER = 0.75;

    @Override
    public void runOpMode(){

        defineComponentCSTB();

        CSTB_Drivetrain CSTB = new CSTB_Drivetrain(mFL, mFR, mBL, mBR);

                waitForStart();

        while(opModeIsActive()){

            double gP1sLy = gamepad1.left_stick_y;
            double gP1sLx = -gamepad1.left_stick_x;
            double gP1sRx = gamepad1.right_stick_x;

            CSTB.CSTBDrive(gP1sLy,gP1sLx,-gP1sRx);
            double FLPOWER = (gP1sLy+gP1sLx+gP1sRx)*POWER_MULTIPLIER;
            double FRPOWER = (gP1sLy-gP1sLx-gP1sRx)*POWER_MULTIPLIER;
            double BLPOWER = (gP1sLy-gP1sLx+gP1sRx)*POWER_MULTIPLIER;
            double BRPOWER = (gP1sLy+gP1sLx-gP1sRx)*POWER_MULTIPLIER;

            telemetry.addData("Front_Left", FLPOWER);
            telemetry.addData("Front_Right", FRPOWER);
            telemetry.addData("Back_Left", BLPOWER);
            telemetry.addData("Back_Right", BRPOWER);
            telemetry.update();


        }
    }
}
