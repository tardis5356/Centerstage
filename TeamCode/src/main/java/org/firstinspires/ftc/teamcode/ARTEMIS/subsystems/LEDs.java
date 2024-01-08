package org.firstinspires.ftc.teamcode.ARTEMIS.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LEDs extends SubsystemBase {

    // setup physical stoof
    private ColorSensor colorLeft, colorRight;
    private RevBlinkinLedDriver blinkin;

    ElapsedTime runtime = new ElapsedTime();

    double EvenOrOdd = 0;

    Gamepad driver, manipulator;

    // setup colors
    public RevBlinkinLedDriver.BlinkinPattern
//            IdleAnim = RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES,
            IdleAnim = RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES,
            RedBlink = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED,
            Red = RevBlinkinLedDriver.BlinkinPattern.RED,
            Yellow = RevBlinkinLedDriver.BlinkinPattern.GOLD,
            Purple = RevBlinkinLedDriver.BlinkinPattern.VIOLET,
            White = RevBlinkinLedDriver.BlinkinPattern.WHITE,
            Green = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;
    private RevBlinkinLedDriver.BlinkinPattern PixColor1 = Purple;
    private RevBlinkinLedDriver.BlinkinPattern PixColor2 = Purple;


    // setup the switch variable
    static String LEDstate = "Idle";

    // map physical objects
    public LEDs(HardwareMap hardwareMap) {
        colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        colorRight = hardwareMap.get(ColorSensor.class, "colorRight");
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        colorLeft.enableLed(false);
        colorRight.enableLed(false);
//        driver = ;
    }

    @Override

    // within periodic, we run a switch statement that will enact the 'modes' of the LEDs.
    // it acts as the trigger and procedural code for swaping the colors
    public void periodic() {
        switch (LEDstate.toLowerCase()) {
            //triggers the Idle set
            case "idle":
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);
                break;
            //triggers the Intakeing set and has procedural based on the color sensor readings to enact different colors.
            case "intaking":
                if (checkLeftPixel() && !checkRightPixel() || !checkLeftPixel() && checkRightPixel()) {
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                } else if (checkLeftPixel() && checkRightPixel()) {
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
                } else if (!checkLeftPixel() && !checkRightPixel()) {
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                }
                break;
            case "yellow":
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                break;
            case "purple":
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                break;
        }
    }


    // This method allows other java classes to pass a state to LEDstate.
    // It allows other classes to use the swich from periodic.
    public void setLEDstate(String state) {
        LEDstate = state;
    }

    // The following two methods use the color sensor to detect a pixel based on distance
    // They then returns if a pixel is there.
    public boolean checkLeftPixel() {
        // sets up a double variable only within the method.
        // also runs the color sensor as a distance sensor
        // gets distance in centimeters, the best unit, and assigns the reading to the variable
        double LeftDistance = ((DistanceSensor) colorLeft).getDistance(DistanceUnit.CM);

        // procedural code, if the variable is below a certain value, then the method returns that there is a pixel.
        // if not, then it returns there is no pixel
        if (LeftDistance < 1.5) {
            return true;
        } else {
            return false;
        }
    }

    // Same as the last method.
    public boolean checkRightPixel() {
        double RightDistance = ((DistanceSensor) colorRight).getDistance(DistanceUnit.CM);

        if (RightDistance < 1.5) {
            return true;
        } else {
            return false;
        }
    }

}
