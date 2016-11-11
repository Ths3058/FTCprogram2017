package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Created by Robotics.
 *
 * @author Ryan Kirkpatrick
 * @version 11/5/2016
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    // Motors
    DcMotor left;
    DcMotor right;
    DcMotor lift;
    DcMotor caplift;
    DcMotor shoot_left;
    DcMotor shoot_right;

    // Servos
    CRServo sweep;
    Servo arm;

    private float leftY = 0;
    private float rightY = 0;

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
        arm = hardwareMap.servo.get("arm");

        //reverse the right motor
        right.setDirection(DcMotor.Direction.REVERSE);
        shoot_right.setDirection(DcMotor.Direction.REVERSE);

        //set the initial positions for the servos
        arm.setPosition(0);

        //set the initial power for the CRServos(continuous rotation servos)
        sweep.setPower(0);
    }

    @Override
    public void loop() {
        //get the values from the gamepads
        //note: pushing the stick all the way up returns -1,
        // so we need to reverse the values
        if (gamepad1.left_stick_y < .1 && gamepad1.left_stick_y > -.1) { //deadband left stick
            leftY = 0;
        } else {
            leftY = -gamepad1.left_stick_y;
        }
        if (gamepad1.right_stick_y < .1 && gamepad1.right_stick_y > -.1) { //deadband right stick
            rightY = 0;
        } else {
            rightY = -gamepad1.right_stick_y;
        }

        //set the power of the motors with the gamepad values
        left.setPower(leftY * 1);
        right.setPower(rightY * 1);

        telemetry.addData("Left Power", leftY);
        telemetry.addData("Right Power", rightY);

        // Vertical Ball Elevator and Sweeper
        if (gamepad1.right_bumper) {
            lift.setPower(-.5);             // elevator up
            sweep.setPower(-1);             // sweeper in
        } else {
            if (gamepad1.left_bumper) {
                lift.setPower(.5);          // elevator down
                sweep.setPower(1);          // sweeper out
            } else {
                lift.setPower(0);           // elevator stop
                sweep.setPower(0);          // sweeper stop
            }
        }

        // Ball Shooter
        if (gamepad1.right_trigger > 0) {
            shootspeed(.9);
        } else {
            shootspeed(0);
        }

        // Cap ball lift
        if (gamepad2.right_trigger > 0) {
            caplift.setPower(.75);
        } else if (gamepad2.left_trigger > 0) {
            caplift.setPower(-.75);
        } else {
            caplift.setPower(0);
        }

        // Arm for holding cap ball lift arm up
        if (gamepad2.a) {
            arm.setPosition(0);
        } else if (gamepad2.b) {
            arm.setPosition(130);
        }
    }

    //----------------------------------
    // Set shooter speed
    //----------------------------------
    private void shootspeed(double speed) {
        shoot_left.setPower(speed);
        shoot_right.setPower(speed);
    }
}