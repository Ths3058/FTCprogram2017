package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Robotics on 11/10/2015.
 *
 * @author Ryan Kirkpatrick
 * @version 1/29/2016
 */
public class Autonomous extends OpMode
{
    // A list of system States.
    enum State
    {
        Fwd1,
        //turnLeft45,
        //Fwd2,
        done
    }

    //--------------------------------------------------------------------------
    // Robot device Objects
    //--------------------------------------------------------------------------
    DcMotor left_back;
    DcMotor left_front;
    DcMotor right_back;
    DcMotor right_front;

    private int         LeftEncoderTarget;
    private int         RightEncoderTarget;

    // Loop cycle time stats variables
    public ElapsedTime mRuntime = new ElapsedTime();   // Time into round.

    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state

    //private State       state;    // Current State Machine State.
    private State       mCurrentState;

    static final int Fwd1count = 12382; // 4.5 feet
    static final int turnLeft45count = 1500;
    static final int Fwd2count = 2160;

    final static double COUNTS = Fwd1count + turnLeft45count + Fwd2count;

    @Override
    public void init()
    {
        left_back = hardwareMap.dcMotor.get("left_back");
        left_front = hardwareMap.dcMotor.get("left_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        right_front = hardwareMap.dcMotor.get("right_front");
        right_back.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.REVERSE);

        setDrivePower(0, 0);
        resetDriveEncoders();
    }

    //--------------------------------------------------------------------------
    // loop
    //--------------------------------------------------------------------------
    // @Override
    public void init_loop()
    {
        // Keep resetting encoders and show the current values
        resetDriveEncoders();        // Reset Encoders to Zero
        telemetry.addData("ENC", String.format("L:R %d:%d", getLeftPosition(), getRightPosition()));
    }

    //--------------------------------------------------------------------------
    // start
    //--------------------------------------------------------------------------
    @Override
    public void start()
    {
        // Setup Robot devices, set initial state and start game clock
        setDriveSpeed(0, 0);        // Set target speed to zero
        runToPosition();            // Run to Position set by encoder targets
        mRuntime.reset();           // Zero game clock
        newState(State.Fwd1);       // starting state
    }

    @Override
    public void loop()
    {
        telemetry.addData("ENC", String.format("L:R %d:%d", getLeftPosition(), getRightPosition()));
        telemetry.addData("Total Target", COUNTS); // shows on screen

        switch(mCurrentState) {
            case Fwd1:
                if (getLeftPosition() > Fwd1count)  {
                    setDriveSpeed(0, 0);
                    resetDriveEncoders();
                    newState(State.done);
                }
                break;
/*
            case turnLeft45:
                if (getLeftPosition() > turnLeft45count) {
                    setDriveSpeed(0, 0);
                    resetDriveEncoders();
                    state = State.Fwd2;
                }
                break;

            case Fwd2:
                if (getLeftPosition() > Fwd2count) {
                    setDriveSpeed(0, 0);
                    resetDriveEncoders();
                    state = State.done;
                }
                break;
*/
            case done:
                break;
        }

        switch(mCurrentState) {
            case Fwd1:
                setDriveSpeed(0.5, 0.5);
                telemetry.addData("Motor Target", Fwd1count); // shows on screen
                setEncoderTarget(Fwd1count, Fwd1count);
                break;
/*
            case turnLeft45:
                setDriveSpeed(0.7, 0.7);
                telemetry.addData("Motor Target", turnLeft45count); // shows on screen
                setEncoderTarget(turnLeft45count, turnLeft45count);
                break;

            case Fwd2:
                setDriveSpeed(0.5, 0.5);
                telemetry.addData("Motor Target", Fwd2count); // shows on screen
                setEncoderTarget(Fwd2count, Fwd2count);
                break;
*/
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
        useConstantPower();
        setDrivePower(0, 0);
    }

    //--------------------------------------------------------------------------
    // User Defined Utility functions here....
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    // setEncoderTarget( LeftEncoder, RightEncoder);
    // Sets Absolute Encoder Position
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    //  Transition to a new state.
    //--------------------------------------------------------------------------
    private void newState(State newState)
    {
        // Reset the state time, and then change to next state.
        mStateTime.reset();
        mCurrentState = newState;
    }

    //--------------------------------------------------------------------------
    // setEncoderTarget( LeftEncoder, RightEncoder);
    // Sets Absolute Encoder Position
    //--------------------------------------------------------------------------
    void setEncoderTarget(int leftEncoder, int rightEncoder)
    {
        left_front.setTargetPosition(LeftEncoderTarget = leftEncoder);
        right_front.setTargetPosition(RightEncoderTarget = rightEncoder);
    }

    //--------------------------------------------------------------------------
    // addEncoderTarget( LeftEncoder, RightEncoder);
    // Sets relative Encoder Position.  Offset current targets with passed data
    //--------------------------------------------------------------------------
    void addEncoderTarget(int leftEncoder, int rightEncoder)
    {
        left_front.setTargetPosition(LeftEncoderTarget += leftEncoder);
        right_front.setTargetPosition(RightEncoderTarget += rightEncoder);
    }

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
    // setDriveSpeed( LeftSpeed, RightSpeed);
    //--------------------------------------------------------------------------
    void setDriveSpeed(double leftSpeed, double rightSpeed)
    {
        setDrivePower(leftSpeed, rightSpeed);
    }

    //--------------------------------------------------------------------------
    // runToPosition ()
    // Set both drive motors to encoder servo mode (requires encoders)
    //--------------------------------------------------------------------------
    public void runToPosition()
    {
        setDriveMode(DcMotorController.RunMode.RUN_TO_POSITION);
    }

    //--------------------------------------------------------------------------
    // useConstantSpeed ()
    // Set both drive motors to constant speed (requires encoders)
    //--------------------------------------------------------------------------
    public void useConstantSpeed()
    {
        setDriveMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    //--------------------------------------------------------------------------
    // useConstantPower ()
    // Set both drive motors to constant power (encoders NOT required)
    //--------------------------------------------------------------------------
    public void useConstantPower()
    {
        setDriveMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    //--------------------------------------------------------------------------
    // resetDriveEncoders()
    // Reset both drive motor encoders, and clear current encoder targets.
    //--------------------------------------------------------------------------
    public void resetDriveEncoders()
    {
        setEncoderTarget(0, 0);
        setDriveMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    //--------------------------------------------------------------------------
    // syncEncoders()
    // Load the current encoder values into the Target Values
    // Essentially synch's the software with the hardware
    //--------------------------------------------------------------------------
    void synchEncoders()
    {
        //	get and set the encoder targets
        LeftEncoderTarget = left_front.getCurrentPosition();
        RightEncoderTarget = right_front.getCurrentPosition();
    }

    //--------------------------------------------------------------------------
    // setDriveMode ()
    // Set both drive motors to new mode if they need changing.
    //--------------------------------------------------------------------------
    public void setDriveMode(DcMotorController.RunMode mode)
    {
        // Ensure the motors are in the correct mode.
        if (left_front.getChannelMode() != mode)
            left_front.setChannelMode(mode);

        if (right_front.getChannelMode() != mode)
            right_front.setChannelMode(mode);
    }

    //--------------------------------------------------------------------------
    // getLeftPosition ()
    // Return Left Encoder count
    //--------------------------------------------------------------------------
    int getLeftPosition()
    {
        return left_front.getCurrentPosition();
    }

    //--------------------------------------------------------------------------
    // getRightPosition ()
    // Return Right Encoder count
    //--------------------------------------------------------------------------
    int getRightPosition()
    {
        return right_front.getCurrentPosition();
    }

    //--------------------------------------------------------------------------
    // encodersAtZero()
    // Return true if both encoders read zero (or close)
    //--------------------------------------------------------------------------
    boolean encodersAtZero()
    {
        return ((Math.abs(getLeftPosition()) < 5) && (Math.abs(getRightPosition()) < 5));
    }
}
