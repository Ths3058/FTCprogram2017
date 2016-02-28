package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Robotics on 10/22/2015.
 *
 * @author Ryan Kirkpatrick
 * @version 11/21/2015
 */
public class TeleOp extends OpMode
{
    // motors
    DcMotor left_back;
    DcMotor left_front;
    DcMotor right_back;
    DcMotor right_front;
    DcMotor arm_extend;
    DcMotor arm_rotate;
    DcMotor climber_rotate;
    DcMotor climber_extend;

    float leftY = 0;
    float rightY = 0;

    Servo bar_right;
    Servo bar_left;
    Servo fender;

    @Override
    public void init() {
        //get references to the motors from the hardware map
        left_back = hardwareMap.dcMotor.get("left_back");
        left_front = hardwareMap.dcMotor.get("left_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        right_front = hardwareMap.dcMotor.get("right_front");
        arm_extend = hardwareMap.dcMotor.get("arm_extend");
        arm_rotate = hardwareMap.dcMotor.get("arm_rotate");
        climber_rotate = hardwareMap.dcMotor.get("climber_rotate");
        climber_extend = hardwareMap.dcMotor.get("climber_extend");

        //get references to the servos from the hardware map
        bar_right = hardwareMap.servo.get("bar_right");
        bar_left = hardwareMap.servo.get("bar_left");
        fender = hardwareMap.servo.get("fender");

        //reverse the right motor
        right_back.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.REVERSE);

        bar_right.setPosition(.98);
        bar_left.setPosition(.55);
        fender.setPosition(1);
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
        left_back.setPower(leftY*0.5);
        left_front.setPower(leftY*0.5);
        right_back.setPower(rightY*0.5);
        right_front.setPower(rightY*0.5);

        telemetry.addData("Left Power", leftY);
        telemetry.addData("Right Power", rightY);

        // Hanging arm
        // - extend arm
        if (gamepad1.left_bumper)     // left_bumper or right_trigger
        {
            arm_extend.setPower(1);
        } else {
            if (gamepad1.right_bumper)
            {
                arm_extend.setPower(-1);
            } else {
                arm_extend.setPower(0);
            }
        }
        // - rotate arm
        if (gamepad1.a)     // left_bumper or a
        {
            arm_rotate.setPower(.15);
        } else {
            if (gamepad1.b)     // left_trigger or b
            {
                arm_rotate.setPower(-.05);
            } else {
                arm_rotate.setPower(0);
            }
        }

        // climber dumper and v-bar hitter
        // - rotate climber dumper
        if (gamepad2.right_trigger > 0) {
            climber_rotate.setPower(0.2);
        } else {
            if (gamepad2.left_trigger > 0) {
                climber_rotate.setPower(-0.2);
            } else {
                climber_rotate.setPower(0);
            }
        }
        // - extend climber dumper
        if (gamepad2.right_bumper) {
            climber_extend.setPower(-0.2);
        } else {
            if (gamepad2.left_bumper) {
                climber_extend.setPower(0.2);
            } else {
                climber_extend.setPower(0);
            }
        }

        // climber bar to trigger climbers
        // - left
        if (gamepad2.x) {
            bar_right.setPosition(.45);
        } else {
            if (gamepad2.y) {
                bar_right.setPosition(.98);
            }
        }
        // - right
        if (gamepad2.a) {
            bar_left.setPosition(1);
        } else {
            if (gamepad2.b) {
                bar_left.setPosition(.55);
            }
        }

        // controls fender to push balls
        if (gamepad1.x) {
            fender.setPosition(1);
        } else {
            if (gamepad1.y) {
                fender.setPosition(.1);
            }
        }
    }
}

