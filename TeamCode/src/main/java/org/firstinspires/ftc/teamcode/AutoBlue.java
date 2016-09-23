package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Robotics on 2/15/2016.
 *
 * @author Ryan Kirkpatrick
 * @version 2/23/2016
 */

public class AutoBlue extends OpMode {
    enum State {
        FWD1,
        Turn1,
        FWD2,
        Turn2,
        FWD3,
        dump_climbers,
        done
    }

    //--------------------------------------------------------------------------
    // Robot device Objects
    //--------------------------------------------------------------------------
    DcMotor left_back;
    DcMotor left_front;
    DcMotor right_back;
    DcMotor right_front;
    DcMotor climber_rotate;

    Servo bar_right;
    Servo bar_left;
    Servo fender;

    private State state;        // Current State Machine State.

    final static double Fwd1count = distToEnc(24);
    final static double Turn1count = degreesToEnc(45); // 90 degrees = 2860
    final static double Fwd2count = distToEnc(64); // 9 feet = 108 inchs = 2750
    final static double Turn2count = degreesToEnc(45);
    final static double Fwd3count = distToEnc(24);

    // Loop cycle time stats variables
    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state

    double COUNTS = 0;

    @Override
    public void init() {
        //get references to the motors from the hardware map
        left_back = hardwareMap.dcMotor.get("left_back");
        left_front = hardwareMap.dcMotor.get("left_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        right_front = hardwareMap.dcMotor.get("right_front");
        climber_rotate = hardwareMap.dcMotor.get("climber_rotate");
        right_back.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.REVERSE);

        COUNTS = Fwd1count;

        //get references to the servos from the hardware map
        bar_right = hardwareMap.servo.get("bar_right");
        bar_left = hardwareMap.servo.get("bar_left");
        fender = hardwareMap.servo.get("fender");

        bar_right.setPosition(.98);
        bar_left.setPosition(.55);
        fender.setPosition(1);
    }

    @Override
    public void start() {
        state = State.FWD1;
        setDrivePower(0, 0);
        mStateTime.reset();
        /*
        left_front.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        right_front.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        left_front.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        right_front.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);*/
    }

    @Override
    public void loop() {
        // Send the current state info (state and time) back to first line of driver station telemetry.
        telemetry.addData("0", String.format("%4.1f ", mStateTime.time()) + state.toString());
        // Send the current encoder info (encoder counts left and right).
        telemetry.addData("ENC", String.format("L:R %d:%d", getLeftPosition(), getRightPosition()));
        // Send the current total counts.
        telemetry.addData("Total Target", COUNTS);

        // First switch statement
        switch (state) {
            case FWD1:
                telemetry.addData("FWD1", 1);
                if (getLeftPosition() > COUNTS) {
                    setDrivePower(0, 0);
                    COUNTS += Turn1count;
                    mStateTime.reset();
                    state = State.Turn1;
                }
            case Turn1:
                telemetry.addData("Turn1", 2);
                if (getLeftPosition() > COUNTS) {
                    setDrivePower(0, 0);
                    COUNTS += Fwd2count;
                    mStateTime.reset();
                    state = State.FWD2;
                }
            case FWD2:
                telemetry.addData("FWD2", 3);
                if (getLeftPosition() > COUNTS)  {
                    setDrivePower(0, 0);
                    COUNTS += Turn2count;
                    mStateTime.reset();
                    state = State.Turn2;
                }
                break;
            case Turn2:
                telemetry.addData("Turn2", 4);
                if (getLeftPosition() > COUNTS) {
                    setDrivePower(0, 0);
                    COUNTS += Fwd3count;
                    mStateTime.reset();
                    state = State.FWD3;
                }
                break;
            case FWD3:
                telemetry.addData("FWD3", 5);
                if (getLeftPosition() > COUNTS) {
                    setDrivePower(0, 0);
                    mStateTime.reset();
                    state = State.dump_climbers;
                }
            case dump_climbers:
                telemetry.addData("dump_climbers", 6);
                if (mStateTime.time() > 3.0) {
                    setDrivePower(0, 0);
                    climber_rotate.setPower(0);
                    mStateTime.reset();
                    state = State.done;
                }
        }

        switch (state) {
            case FWD1:
                setDrivePower(0.5, 0.5);
                break;
            case Turn1:
                setDrivePower(0.5, -0.5);
                break;
            case FWD2:
                setDrivePower(0.5, 0.5);
                break;
            case Turn2:
                setDrivePower(0.5, -0.5);
                break;
            case FWD3:
                setDrivePower(0.5, 0.5);
                break;
            case dump_climbers:
                climber_rotate.setPower(0.2);
                break;
            case done:
                stop();
                break;
        }
    }

    //--------------------------------------------------------------------------
    // stop
    //--------------------------------------------------------------------------
    @Override
    public void stop()
    {
        // Ensure that the motors are turned off.
        setDrivePower(0, 0);
    }

    //--------------------------------------------------------------------------
    // User Defined Utility functions here....
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    // setDrivePower( LeftPower, RightPower);
    //--------------------------------------------------------------------------
    void setDrivePower(double leftPower, double rightPower)
    {
        left_front.setPower(leftPower);
        left_back.setPower(leftPower);
        right_front.setPower(rightPower);
        right_back.setPower(rightPower);
    }

    //--------------------------------------------------------------------------
    // getLeftPosition ()
    // Return Left Encoder count
    //--------------------------------------------------------------------------
    int getLeftPosition() { return Math.abs(left_front.getCurrentPosition()); }

    //--------------------------------------------------------------------------
    // getRightPosition ()
    // Return Right Encoder count
    //--------------------------------------------------------------------------
    int getRightPosition()
    {
        return Math.abs(right_front.getCurrentPosition());
    }

    //--------------------------------------------------------------------------
    // distToEnc ()
    // Parameters feet
    // Return encoder count
    //--------------------------------------------------------------------------
    static int distToEnc(double inch) { return (int)(inch/12.0*2750); }

    //--------------------------------------------------------------------------
    // degreesToEnc ()
    // Parameters degrees
    // Return encoder count
    //--------------------------------------------------------------------------
    static int degreesToEnc(int degrees) { return (int)(degrees/90.0*2500); } // 2860
}
