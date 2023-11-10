package org.firstinspires.ftc.teamcode.teleop.VR_G1_Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LEDs extends SubsystemBase {

    // setup physical stoof
    private ColorSensor colorLeft, colorRight;
    private RevBlinkinLedDriver blinkin;

    // setup colors
    RevBlinkinLedDriver.BlinkinPattern
            IdleAnim = RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES,
            Yellow  = RevBlinkinLedDriver.BlinkinPattern.GOLD,
            YellowBlink = RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_SLOW,
            Purple = RevBlinkinLedDriver.BlinkinPattern.VIOLET,
            White = RevBlinkinLedDriver.BlinkinPattern.WHITE,
            Green = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN,
            Red = RevBlinkinLedDriver.BlinkinPattern.DARK_RED,
            RedBlink = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;

    // setup the switch variable
    static String LEDstate = "Idle";

    // map physical objects
    public LEDs(HardwareMap hardwareMap){
        colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        colorRight = hardwareMap.get(ColorSensor.class, "colorRight");
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "LEDa");
    }

    @Override

    // within periodic, we run a switch statement that will enact the 'modes' of the LEDs.
    // it acts as the trigger and procedural code for swaping the colors
    public void periodic(){
        switch (LEDstate){
            //triggers the Idle set
            case "Idle" :
                setIdle();
                break;

            //triggers the Signal set and
            case "Signaling" :

                break;

            //triggers the Intakeing set and has procedural based on the color sensor readings to enact different colors.
            case "Intaking" :
                if(checkLeftPixel() && !checkRightPixel() || !checkLeftPixel() && checkRightPixel()) {
                    setOnePixel();
                }
                else if(checkLeftPixel() && checkRightPixel()){
                    setTwoPixels();
                }
                else if(!checkLeftPixel() && !checkRightPixel()){
                    LEDstate = "Signaling";
                }
                break;
        }
    }

    // This method allows other java classes to pass a state to LEDstate.
    // It allows other classes to use the swich from periodic.
    public static void setLEDstate(String state){
        LEDstate = state;
    }

    // Code for the idle animation as well.
    // It isn't necessary to turn off the sensor LEDs,
    // I just thought that that'd turn off the sensor and it doesn't pose any harm.
    public void setIdle(){
        colorLeft.enableLed(false);
        colorRight.enableLed(false);

        blinkin.setPattern(IdleAnim);

    }

    // The following two methods use the color sensor to detect a pixel based on distance
    // They then returns if a pixel is there.
    public boolean checkLeftPixel(){
        colorLeft.enableLed(true);

        // sets up a double variable only within the method.
        // also runs the color sensor as a distance sensor
        // gets distance in centimeters, the best unit, and assigns the reading to the variable
        double LeftDistance = ((DistanceSensor) colorLeft).getDistance(DistanceUnit.CM);

        // procedural code, if the variable is below a certain value, then the method returns that there is a pixel.
        // if not, then it returns there is no pixel
        if (LeftDistance < 1){
            return true;
        }
        else {
            return false;
        }

    }

    // Same as the last method.
    public boolean checkRightPixel(){
        colorRight.enableLed(true);

        double RightDistance = ((DistanceSensor) colorRight).getDistance(DistanceUnit.CM);

        if (RightDistance < 1){
            return true;
        }
        else {
            return false;
        }
    }

    public void setNoPixels(){
        blinkin.setPattern(Green);
    }

    // Method sets color to a blinking gold
    public void setOnePixel(){
        blinkin.setPattern(YellowBlink);
    }

    // Method sets color to a blinking red
    public void setTwoPixels(){
        blinkin.setPattern(RedBlink);
    }

}
