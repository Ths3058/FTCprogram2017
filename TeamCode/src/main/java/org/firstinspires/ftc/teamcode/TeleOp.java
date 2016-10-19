package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Robotics.
 *
 * @author Ryan Kirkpatrick
 * @version 9/23/2016
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
//@Disabled
public class TeleOp extends OpMode {
    // motors
    DcMotor left;
    DcMotor right;
    DcMotor lift;
    /*DcMotor shoot_left;
    DcMotor shoot_right; */

    Servo sweep;

    private float leftY = 0;
    private float rightY = 0;

    @Override
    public void init() {
        //get references to the motors from the hardware map
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        lift = hardwareMap.dcMotor.get("lift");
        /*shoot_left = hardwareMap.dcMotor.get("shoot_left");
        shoot_right = hardwareMap.dcMotor.get("shoot_right");         */

        //get references to the servos from the hardware map
        sweep = hardwareMap.servo.get("sweep");

        //reverse the right motor
        right.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.REVERSE);

        //set the initial positions for the servos
    }

    @Override
    public void loop() {
        //get the values from the gamepads
        //note: pushing the stick all the way up returns -1,
        // so we need to reverse the values
        if (gamepad1.left_stick_y < .1 && gamepad1.left_stick_y > -.1) { //deadband left
            leftY = 0;
        } else {
            leftY = -gamepad1.left_stick_y;
        }
        if (gamepad1.right_stick_y < .1 && gamepad1.right_stick_y > -.1) { //deadband right
            rightY = 0;
        } else {
            rightY = -gamepad1.right_stick_y;
        }

        //set the power of the motors with the gamepad values
        left.setPower(leftY * 0.5);
        right.setPower(rightY * 0.5);

        telemetry.addData("Left Power", leftY);
        telemetry.addData("Right Power", rightY);

        // Vertical Ball Elevator and Sweeper
        if (gamepad1.right_bumper) {
            lift.setPower(-.5);         // elevator up
            //sweep.setPosition(1);
        } else {
            if (gamepad1.left_bumper) {
                lift.setPower(.5);      // elevator down
                //sweep.setPosition(0);
            } else {
                lift.setPower(0);
                //sweep.setPosition(0.5);
            }
        }

        /*// Ball Shooter
        if (gamepad1.right_trigger > 0) {
            shoot(1.0);
        } else {
            shoot(0);
        }     */

        /*// Elevator Sweeper
        if (gamepad1.a) {
            sweep.setPosition(1);
        } else {
            sweep.setPosition(0.5);
        }*/
    }

    //----------------------------------
    // Set shooter speed
    //----------------------------------
    /*private void shoot(double speed) {
        shoot_left.setPower(speed);
        shoot_right.setPower(speed);
    }     */
}