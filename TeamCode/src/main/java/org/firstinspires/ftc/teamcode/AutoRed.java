package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.IOException;

import static org.firstinspires.ftc.teamcode.StaticFunctions.degreesToEnc;
import static org.firstinspires.ftc.teamcode.StaticFunctions.distToEnc;

/**
 * Created by Robotics.
 *
 * @author Ryan Kirkpatrick
 * @version 11/5/2016
 */
@Autonomous(name = "Red1", group = "Autonomous")
public class AutoRed extends OpMode {
    private enum State {
        FWD1,
        Shoot,
        TURNL1,
        FWD2,
        TURNL2,
        FWD3,
        Button1,
        REV1,
        TURNR1,
        FWD4,
        TURNL3,
        FWD5,
        Button2,
        done
    }

    //--------------------------------------------------------------------------
    // Robot device Objects
    //--------------------------------------------------------------------------
    private DcMotor left;
    private DcMotor right;
    private DcMotor lift;
    private DcMotor caplift;
    private DcMotor shoot_left;
    private DcMotor shoot_right;

    private CRServo sweep;
    private Servo arm;
    private Servo button_left;
    private Servo button_right;

    ModernRoboticsI2cRangeSensor rangeSensor;
    ColorSensor colorSensor;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    private State state;        // Current State Machine State.

    //set counts for each state
    private final static double Fwd1count = distToEnc(24);
    private final static double Shoottime = 4; // in seconds
    private final static double TURNL1count = degreesToEnc(45);
    private final static double FWD2count = distToEnc(62);
    private final static double TURNL2count = degreesToEnc(91);
    private final static double FWD3dist = 24; // in cm
    private final static double Button1time = 3.5; // in seconds
    private final static double REV1count = distToEnc(24);
    private final static double TURNR1count = degreesToEnc(75);
    private final static double FWD4count = distToEnc(60);
    private final static double TURNL3count = degreesToEnc(140);
    private final static double FWD5dist = 24; // in cm
    private final static double Button2time = 3.5; //in seconds

    // Loop cycle time stats variables
    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state

    private double COUNTS = 0;

    // bLedOn represents the state of the LED.
    boolean bLedOn = false;

    @Override
    public void init() {
        //get references to the motors from the hardware map
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        lift = hardwareMap.dcMotor.get("lift");
        caplift = hardwareMap.dcMotor.get("caplift");
        shoot_left = hardwareMap.dcMotor.get("shoot_left");
        shoot_right = hardwareMap.dcMotor.get("shoot_right");

        //get references to the servos from the hardware map
        arm = hardwareMap.servo.get("arm");
        sweep = hardwareMap.crservo.get("sweep");
        button_left = hardwareMap.servo.get("button_left");
        button_right = hardwareMap.servo.get("button_right");

        //get references to the sensors from the hardware map
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        colorSensor = hardwareMap.colorSensor.get("sensor_color");

        //reverse the right motor
        right.setDirection(DcMotor.Direction.REVERSE);
        shoot_right.setDirection(DcMotor.Direction.REVERSE);

        //set the initial positions for the servos
        arm.setPosition(0);
        button_right.setPosition(0);
        button_left.setPosition(0);

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);

        COUNTS = Fwd1count;
        /*if (getRightPosition() > 0 || getLeftPosition() > 0) {
            throw new Error("Restart robot to reset encoders.");
        }*/
    }

    @Override
    public void start() {
        state = State.FWD1;
        setDrivePower(0, 0);
        mStateTime.reset();
    }

    @Override
    public void loop() {
        // convert the RGB values to HSV values.
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        // Send the current state info (state and time) back to first line of driver station telemetry.
        telemetry.addData("0", String.format("%4.1f %6s", mStateTime.time(), state.toString()));
        // Send the current encoder info (encoder counts left and right).
        telemetry.addData("ENC", String.format("L:R %d:%d", getLeftPosition(), getRightPosition()));
        // Send the current total counts.
        telemetry.addData("Total Target ", COUNTS);

        // send the info back to driver station using telemetry function.
        telemetry.addData("LED", bLedOn ? "On" : "Off");
        //telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        //telemetry.addData("Hue", hsvValues[0]);

        //telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
        //telemetry.addData("raw optical", rangeSensor.rawOptical());
        telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();

        // First switch statement
        switch (state) {
            case FWD1:
                if (getRightPosition() > COUNTS) {
                    setDrivePower(0, 0);
                    mStateTime.reset();
                    state = State.Shoot;
                }
                break;
            case Shoot:
                if (mStateTime.time() > Shoottime) {
                    shootspeed(0);
                    lift.setPower(0);
                    COUNTS = getRightPosition() + TURNL1count;
                    mStateTime.reset();
                    state = State.TURNL1;
                }
                break;
            case TURNL1:
                if (getRightPosition() > COUNTS) {
                    setDrivePower(0, 0);
                    COUNTS = getRightPosition() + FWD2count;
                    mStateTime.reset();
                    state = State.FWD2;
                }
                break;
            case FWD2:
                if (getLeftPosition() > COUNTS) {
                    setDrivePower(0, 0);
                    COUNTS = getLeftPosition() + TURNL2count;
                    mStateTime.reset();
                    state = State.TURNL2;
                }
                break;
            case TURNL2:
                if (getLeftPosition() > COUNTS) {
                    setDrivePower(0, 0);
                    mStateTime.reset();
                    state = State.FWD3;
                }
                break;
            case FWD3:
                if (rangeSensor.getDistance(DistanceUnit.CM) < FWD3dist) {
                    setDrivePower(0, 0);
                    mStateTime.reset();
                    state = State.Button1;
                }
                break;
            case Button1:
                if (mStateTime.time() > Button1time) {
                    COUNTS = getRightPosition() - REV1count;
                    mStateTime.reset();
                    state = State.REV1;
                }
                break;
            case REV1:
                if (getRightPosition() < COUNTS) {
                    setDrivePower(0, 0);
                    COUNTS = getLeftPosition() + TURNR1count;
                    mStateTime.reset();
                    state = State.TURNR1;
                }
                break;
            case TURNR1:
                if (getLeftPosition() > COUNTS) {
                    setDrivePower(0, 0);
                    COUNTS = getRightPosition() + FWD4count;
                    mStateTime.reset();
                    state = State.FWD4;
                }
                break;
            case FWD4:
                if (getRightPosition() > COUNTS) {
                    setDrivePower(0, 0);
                    COUNTS = getRightPosition() + TURNL3count;
                    mStateTime.reset();
                    state = State.TURNL3;
                }
                break;
            case TURNL3:
                if (getRightPosition() > COUNTS) {
                    setDrivePower(0, 0);
                    mStateTime.reset();
                    state = State.FWD5;
                }
                break;
            case FWD5:
                if (rangeSensor.getDistance(DistanceUnit.CM) < FWD5dist) {
                    setDrivePower(0, 0);
                    mStateTime.reset();
                    state = State.Button2;
                }
                break;
            case Button2:
                if (mStateTime.time() > Button2time) {
                    mStateTime.reset();
                    state = State.done;
                }
                break;
        }

        switch (state) {
            case FWD1:
                setDrivePower(0.5, 0.5);
                break;
            case Shoot:
                shootspeed(.8);
                lift.setPower(.25);
                break;
            case TURNL1:
                setDrivePower(-0.35,0.35);
                break;
            case FWD2:
                setDrivePower(0.5,0.5);
                break;
            case TURNL2:
                setDrivePower(-0.35,0.35);
                break;
            case FWD3:
                setDrivePower(0.5,0.5);
                break;
            case Button1:
                if (colorSensor.blue() > 6) {
                    button_left.setPosition(180);
                } else if (colorSensor.red() > 3) {
                    button_right.setPosition(180);
                }
                if (mStateTime.time() > 2.5) {
                    button_left.setPosition(0);
                    button_right.setPosition(0);
                }
                break;
            case REV1:
                setDrivePower(-0.5,-0.5);
                break;
            case TURNR1:
                setDrivePower(0.35,-0.35);
                break;
            case FWD4:
                setDrivePower(0.5,0.5);
                break;
            case TURNL3:
                setDrivePower(-0.35,0.35);
                break;
            case FWD5:
                setDrivePower(0.5,0.5);
                break;
            case Button2:
                if (colorSensor.blue() > 6) {
                    button_left.setPosition(180);
                } else if (colorSensor.red() > 3) {
                    button_right.setPosition(180);
                }
                if (mStateTime.time() > 2.5) {
                    button_left.setPosition(0);
                    button_right.setPosition(0);
                }
                break;
            case done:
                stop();
                break;
        }
    }

    //--------------------------------------------------------------------------
    // User Defined Utility functions here....
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    // stop
    //--------------------------------------------------------------------------
    @Override
    public void stop() {
        // Ensure that the motors are turned off.
        setDrivePower(0, 0);
        caplift.setPower(0);
        lift.setPower(0);
        shootspeed(0);
    }

    //--------------------------------------------------------------------------
    // setDrivePower( LeftPower, RightPower );
    //--------------------------------------------------------------------------
    private void setDrivePower(double leftPower, double rightPower)
    {
        left.setPower(leftPower);
        right.setPower(rightPower);
    }

    //----------------------------------
    // shoot ( power );
    // Set shooter speed
    //----------------------------------
    private void shootspeed(double speed) {
        shoot_left.setPower(speed);
        shoot_right.setPower(speed);
    }

    //--------------------------------------------------------------------------
    // getLeftPosition ()
    // Return Left Encoder count
    //--------------------------------------------------------------------------
    private int getLeftPosition() {
        return Math.abs(left.getCurrentPosition());
    }

    //--------------------------------------------------------------------------
    // getRightPosition ()
    // Return Right Encoder count
    //--------------------------------------------------------------------------
    private int getRightPosition()
    {
        return Math.abs(right.getCurrentPosition());
    }
}
