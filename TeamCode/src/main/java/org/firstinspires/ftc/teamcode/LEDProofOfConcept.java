package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="LEDs Babyyyyyyyyyyyyy")
public class LEDProofOfConcept extends LinearOpMode {
    //Servo LED;

    ElapsedTime runtime = new ElapsedTime();

//    public void resetRuntime() {
//        super.resetRuntime();
//    }

    RevBlinkinLedDriver blinkin;

    double PixelCount = 0;
    double PixelColor = 0;
    double PixelPrev = 0;
    double EvenOrOdd = 0;

    RevBlinkinLedDriver.BlinkinPattern TardisPurple;
    RevBlinkinLedDriver.BlinkinPattern Yellow;
    RevBlinkinLedDriver.BlinkinPattern Green;
    RevBlinkinLedDriver.BlinkinPattern Red;

//    enum BlinkinColors {
//        BLUE_VIOLET,
//        YELLOW,
//        DARK_GREEN,
//        DARK_RED;
//    }

   // BlinkinColors blinkinColor;

    @Override
    public void runOpMode(){


//        LED = hardwareMap.get(Servo.class, "blinkin");
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        boolean released = true;

//        TardisPurple = RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET;
//        Yellow = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
//        Green = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;
//        Red = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;

        waitForStart();
        while(opModeIsActive()){

            runtime.seconds();
            int currentRunTime = (int) runtime.seconds();

            if (currentRunTime%2 == 0){
                EvenOrOdd = 0;
            }
            else {
                EvenOrOdd = 1;
            }


            boolean Up = gamepad1.dpad_up;
            boolean Down =gamepad1.dpad_down;
            boolean A = gamepad1.a;
            boolean B = gamepad1.b;
            boolean Y = gamepad1.y;
            boolean X = gamepad1.x;
            boolean RB = gamepad1.right_bumper;


//            switch(blinkinColor){
//                case YELLOW:
//                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//                    break;
//            }

//            blinkinColor = BlinkinColors.YELLOW;

            if(RB == true){
                PixelPrev = PixelColor;
            }

            if (A){
                PixelColor = 0;
            }
            else if (B){
                PixelColor = 1;
            }
            else if (X){
                PixelColor = 2;
            }
            else if (Y){
                PixelColor = 3;
            }


            if (Up && released) {
                PixelCount += 1;
                released = false;
            }
            else if (Down && released){
                PixelCount -= 1;
                released = false;
            }
            else if (Up == false && Down == false){
                released = true;
            }

            if (PixelCount == 0){
//              blinkinColor = BlinkinColors.BLUE_VIOLET;
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);
//
            }


            else if (PixelCount == 1 && EvenOrOdd == 0) {
                if (PixelColor == 0){
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);

                }
                else if (PixelColor == 1){
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);

                }
                else if (PixelColor == 2){
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);

                }
                else if (PixelColor == 3){
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);

                }

            }


            else if (PixelCount == 1 && EvenOrOdd == 1) {
                if (PixelPrev == 0){
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);

                }
                else if (PixelPrev == 1){
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);

                }
                else if (PixelPrev == 2){
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);

                }
                else if (PixelPrev == 3){
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);

                }

            }



            else if (PixelCount == 2){
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_SLOW);

            }
            else if (PixelCount == 3){
//                blinkinColor = BlinkinColors.DARK_RED;
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);

            }
//

            telemetry.addData("PixelPrev", PixelPrev);
            telemetry.addData("PixelColor", PixelColor);
            telemetry.addData("PixelCount", PixelCount-1);
            telemetry.update();
        }
    }

}
