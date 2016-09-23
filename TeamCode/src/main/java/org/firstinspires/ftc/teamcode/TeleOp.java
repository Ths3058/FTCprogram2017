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
@Disabled
public class TeleOp extends OpMode
{
    // motors
    DcMotor left;
    DcMotor right;

    float leftY = 0;
    float rightY = 0;

    @Override
    public void init() {
        //get references to the motors from the hardware map
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");

        //get references to the servos from the hardware map

        //reverse the right motor
        right.setDirection(DcMotor.Direction.REVERSE);

        //set the initial positions for the servos
    }

    @Override
    public void loop() {
        //get the values from the gamepads
        //note: pushing the stick all the way up returns -1,
        // so we need to reverse the values
        if(gamepad1.left_stick_y < .1 && gamepad1.left_stick_y > -.1) { //deadband left
            leftY = 0;
        } else {
            leftY = -gamepad1.left_stick_y;
        }
        if(gamepad1.right_stick_y < .1 && gamepad1.right_stick_y > -.1) { //deadband right
            rightY = 0;
        } else {
            rightY = -gamepad1.right_stick_y;
        }

        //set the power of the motors with the gamepad values
        left.setPower(leftY * 0.5);
        right.setPower(rightY * 0.5);

        telemetry.addData("Left Power", leftY);
        telemetry.addData("Right Power", rightY);
    }
}

