package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Robotics.
 *
 * @author Ryan Kirkpatrick
 * @version 2/23/2016
 */
@Autonomous(name = "Auto:Blue", group = "Autonomous")
@Disabled
public class AutoBlue extends OpMode {
    enum State {
        FWD1,
        done
    }

    //--------------------------------------------------------------------------
    // Robot device Objects
    //--------------------------------------------------------------------------
    DcMotor left;
    DcMotor right;

    private State state;        // Current State Machine State.

    //set counts for each state
    final static double Fwd1count = distToEnc(24);
    final static double Turn1count = degreesToEnc(45); // 90 degrees = 2860
    final static double Fwd2count = distToEnc(47.5); // 9 feet = 108 inchs = 2750
    final static double Turn2count = degreesToEnc(45);
    final static double Fwd3count = distToEnc(24);

    // Loop cycle time stats variables
    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state

    double COUNTS = 0;

    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");

        COUNTS = Fwd1count;
    }

    @Override
    public void start() {
        state = State.FWD1;
        setDrivePower(0, 0);
        mStateTime.reset();

        //get references to the servos from the hardware map

        //set initial servo positions
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
                if (getRightPosition() > COUNTS) {
                    setDrivePower(0, 0);
                    COUNTS += Turn1count;
                    mStateTime.reset();
                    state = State.done;
                }
                break;
        }

        switch (state) {
            case FWD1:
                setDrivePower(0.5, 0.5);
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
        left.setPower(leftPower);
        right.setPower(rightPower);
    }

    //--------------------------------------------------------------------------
    // getLeftPosition ()
    // Return Left Encoder count
    //--------------------------------------------------------------------------
    int getLeftPosition() {
        return Math.abs(left.getCurrentPosition());
    }

    //--------------------------------------------------------------------------
    // getRightPosition ()
    // Return Right Encoder count
    //--------------------------------------------------------------------------
    int getRightPosition()
    {
        return Math.abs(right.getCurrentPosition());
    }

    //--------------------------------------------------------------------------
    // distToEnc ()
    // Parameters inches
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
