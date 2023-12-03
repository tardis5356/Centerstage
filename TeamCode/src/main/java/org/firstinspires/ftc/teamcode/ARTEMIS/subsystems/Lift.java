package org.firstinspires.ftc.teamcode.ARTEMIS.subsystems;

import static org.firstinspires.ftc.teamcode.ARTEMIS.subsystems.BotPositions.DISTANCE_FROM_BACKDROP;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Lift extends SubsystemBase {
    private PIDController controller;
    private DcMotorEx mLR, mLL;
    private DcMotorEx mFL, mFR, mBL, mBR;

    private TouchSensor liftBase;

    private DistanceSensor distanceBackRight, distanceBackLeft;

    public static double p = BotPositions.LIFT_p, i = BotPositions.LIFT_i, d = BotPositions.LIFT_d;
    public static double ff = BotPositions.LIFT_ff;

    public static int target = 0;

    public static int tolerance = BotPositions.LIFT_TOLERANCE;

    public double power = 0;
    public double stickValue = 0;
    public double stickValue2 = 0;
    public double pid = 0;
    public boolean manualActive = false;
    
    public int resets = 0;

    public int liftOffset = 0;

    // exp hub i2c
    //      0 right distance
    //      1 left distance

    public Lift(HardwareMap hardwareMap) {
        mLR = hardwareMap.get(DcMotorEx.class, "mLR");
        mLL = hardwareMap.get(DcMotorEx.class, "mLL");

        controller = new PIDController(p, i, d);

        liftBase = hardwareMap.get(TouchSensor.class, "touchLift");
        distanceBackRight = hardwareMap.get(DistanceSensor.class, "distanceBackRight");
        distanceBackLeft = hardwareMap.get(DistanceSensor.class, "distanceBackLeft");

        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");

        mLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mLL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mLR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mLL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mLR.setDirection(DcMotorSimple.Direction.REVERSE);
        mLL.setDirection(DcMotorSimple.Direction.REVERSE);

        target = -10; // lift should start fully retracted
    }

    public void periodic() {
        liftPID_R2V2();
        //if lift is at the base (based on touch sensor) and absolute value of lift position is greater than __ number of ticks (it has drifted by a significant amount), reset position at base
        if (liftBase.isPressed() && Math.abs(mLR.getCurrentPosition()) > tolerance) {
            mLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mLR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                resets++; // to test if the lift position is being reset
        }
    }
    
    public void setTargetPosition(int targetPos) {
        target = targetPos; // set the target position
        manualActive = false; // lift is not being moved manually anymore
    }

    public void setLiftOffset() {
        if (mLR.getCurrentPosition() > 300 && liftOffset == 0) { // if lift is extended and no offset has been set already
            liftOffset = 1;
        }
        if (liftOffset == 1 && liftBase.isPressed()) { // once lift has extended once and limit is reached, get offset
            liftOffset = mLR.getCurrentPosition();
        }
    }

    public void manualControl(double stick, double stick2) {
        // macro adjustment
        stickValue = stick * 1;

        // micro adjustment
        if (stickValue2 < 0) stickValue2 = stick2 * 0.2; // when lowering, move at 20% speed
        else stickValue2 = stick2 * 0.4; // when raising, move at 40% speed
    }
    
    public void liftPID_R2V2() {
        int liftPos = mLR.getCurrentPosition();
        int liftTarget = target; // + liftOffset;
        double pidController = controller.calculate(liftPos, target);
        pid = pidController; // for getLiftPID()

        if (!manualActive) {
            // lowest position (no motor power) target position is -10
            if (liftTarget != -10)
                power = pid + ff; // if lift is not supposed to be fully retracted, motor power is feedforward (counteract gravity) + value from pid controller

            if (liftTarget == -10 && !liftBase.isPressed())
                power = 0.1; // if lift is supposed to be retracted but the lift base touch sensor is not pressed, motors should lower
            if (liftTarget == -10 && liftBase.isPressed())
                power = 0; // if lift is supposed to be retracted and lift base touch sensor is pressed, motors should be stopped

            // continuously update power to motors based on pid value
            mLR.setPower(power);
            mLL.setPower(power);

            if (Math.abs(stickValue + stickValue2) > 0.05) {
                manualActive = true; // if the values of the sticks is not 0 (aka they are moved), activate manual mode, which adds the
            }
        }

        if (manualActive) { // if manual is active, set motor power to feedforward (counteract gravity), and the stick values (both micro and macro adjustment)
            power = ff + stickValue + stickValue2;
            mLR.setPower(power);
            mLL.setPower(power);
        }
    }

    public double scaleBetween(double inputMin, double inputMax, double scaledMin, double scaledMax, double inputInput) {
        return (inputInput - inputMin)/(inputMax - inputMin) * (scaledMax - scaledMin) + scaledMin;
    }

    public void squareToBackdrop(){
        double distRight = distanceBackRight.getDistance(DistanceUnit.CM);
        double distLeft = distanceBackLeft.getDistance(DistanceUnit.CM);

        double rightError = DISTANCE_FROM_BACKDROP - distRight; // positive = far away, negative = too close
        double leftError = DISTANCE_FROM_BACKDROP - distLeft;
        double error = distRight-distLeft;

        // if error is positive, right is further away than left
        // if error is negative, left is further away than right
        // target position is DISTANCE_FROM_BACKDROP

        double MAX_POWER = 0.5, MIN_POWER = 0.2;

        double leftPower = 0, rightPower = 0;

        double rightPowerScaled = scaleBetween(0, 40, MIN_POWER, MAX_POWER, rightError);
        double leftPowerScaled = scaleBetween(0, 40, MIN_POWER, MAX_POWER, leftError);

        if(leftError > 40) {
            leftPower = -0.5;
        } else if(leftError >= 2) {
            leftPower = leftPowerScaled;
        } else if (leftError <= -2){
            leftPower = -leftPowerScaled;
        }

        if(rightError > 40) {
            rightPower = -0.5;
        } else if(rightError >= 2) {
            rightPower = rightPowerScaled;
        } else if (rightError <= -2){
            rightPower = -rightPowerScaled;
        }


        mFL.setPower(leftPower);
        mBL.setPower(leftPower);
        mFR.setPower(rightPower);
        mBR.setPower(rightPower);
    }

    public boolean getLiftBase() {
        return liftBase.isPressed();
    }

    public double getDistanceBackRight() {
        return distanceBackRight.getDistance(DistanceUnit.CM);
    }
    public double getDistanceBackLeft() {
        return distanceBackLeft.getDistance(DistanceUnit.CM);
    }

    public int getLiftBaseResets() {
        return resets;
    }

    public double getLiftPosition() {
        return mLR.getCurrentPosition();
    }

    public double getLiftPower() {
        return power;
    }

    public double getLiftPID() {
        return pid;
    }

    public double getLiftFF() {
        return ff;
    }

    public double getLiftTargetPosition() {
        return target;
    }
    
    public boolean atLimit() {
        return getLiftPosition() > 650 || getLiftPosition() < 5;
    }

}
