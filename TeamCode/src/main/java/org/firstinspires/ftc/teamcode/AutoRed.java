package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Robotics.
 *
 * @author Ryan Kirkpatrick
 * @version 11/5/2016
 */
@Autonomous(name = "Auto:Red", group = "Autonomous")
public class AutoRed extends OpMode {
    enum State {
        FWD1,
        Shoot,
        FWD2,
        done
    }

    //--------------------------------------------------------------------------
    // Robot device Objects
    //--------------------------------------------------------------------------
    DcMotor left;
    DcMotor right;
    DcMotor lift;
    DcMotor caplift;
    DcMotor shoot_left;
    DcMotor shoot_right;

    CRServo sweep;

    private State state;        // Current State Machine State.

    //set counts for each state
    final static double Fwd1count = distToEnc(24);
    final static double Fwd2count = distToEnc(24);

    // Loop cycle time stats variables
    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state

    double COUNTS = 0;

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
        sweep = hardwareMap.crservo.get("sweep");

        //reverse the right motor
        right.setDirection(DcMotor.Direction.REVERSE);
        shoot_right.setDirection(DcMotor.Direction.REVERSE);

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
                    mStateTime.reset();
                    state = State.Shoot;
                }
                break;
            case Shoot:
                telemetry.addData("Shoot", 1);
                if (mStateTime.time() >= 5) {
                    shoot(0);
                    COUNTS += Fwd2count;
                    mStateTime.reset();
                    state = State.FWD2;
                }
                break;
            case FWD2:
                telemetry.addData("FWD2", 1);
                if (getLeftPosition() > COUNTS) {
                    setDrivePower(0, 0);
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
                shoot(.9);
                lift.setPower(-.25);
                break;
            case FWD2:
                setDrivePower(0.5, 0.5);
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
    public void stop()
    {
        // Ensure that the motors are turned off.
        setDrivePower(0, 0);
        caplift.setPower(0);
        lift.setPower(0);
        shoot(0);
    }

    //--------------------------------------------------------------------------
    // setDrivePower( LeftPower, RightPower );
    //--------------------------------------------------------------------------
    void setDrivePower(double leftPower, double rightPower)
    {
        left.setPower(leftPower);
        right.setPower(rightPower);
    }

    //----------------------------------
    // shoot ( power );
    // Set shooter speed
    //----------------------------------
    private void shoot(double speed) {
        shoot_left.setPower(speed);
        shoot_right.setPower(speed);
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
    static int distToEnc(double inch) { return (int)(inch/12.0*450); } //2750

    //--------------------------------------------------------------------------
    // degreesToEnc ()
    // Parameters degrees
    // Return encoder count
    //--------------------------------------------------------------------------
    static int degreesToEnc(int degrees) { return (int)(degrees/90.0*450); } // 2860, 2500
}
