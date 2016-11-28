package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Robotics.
 *
 * @author Ryan Kirkpatrick
 * @version 11/5/2016
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    // Motors
    private DcMotor left;
    private DcMotor right;
    private DcMotor lift;
    private DcMotor caplift;
    private DcMotor shoot_left;
    private DcMotor shoot_right;

    // Servos
    private CRServo sweep;
    //private Servo arm;
    private Servo arm_left;
    private Servo arm_right;

    // Variavles
    private float leftY = 0;
    private float rightY = 0;
    private boolean capOpen = false;
    private boolean capClose = false;

    // Loop cycle time stats variables
    private ElapsedTime buttonTime = new ElapsedTime();  // Time into current state

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
        //arm = hardwareMap.servo.get("arm");
        arm_left = hardwareMap.servo.get("arm_left");
        arm_right = hardwareMap.servo.get("arm_right");

        //reverse the right motor
        right.setDirection(DcMotor.Direction.REVERSE);
        shoot_right.setDirection(DcMotor.Direction.REVERSE);

        //set the initial positions for the servos
        //arm.setPosition(0);
        arm_left.setPosition(0);
        arm_right.setPosition(0);

        //set the initial power for the CRServos(continuous rotation servos)
        sweep.setPower(0);
    }

    @Override
    public void loop() {
        //get the values from the gamepads
        //note: pushing the stick all the way up returns -1,
        // so we need to reverse the values
        //Gamepad 1 and 2 Drive
        if (gamepad1.left_stick_y > .1 || gamepad1.left_stick_y < -.1) { //Gamepad 1 deadband left stick
            leftY = -gamepad1.left_stick_y;
        } else if (gamepad2.right_stick_y > .1 || gamepad2.right_stick_y < -.1) { //Gamepad 2 deadband right stick
            leftY = (gamepad2.right_stick_y)/2;
        } else {
            leftY = 0;
        }
        if (gamepad1.right_stick_y > .1 || gamepad1.right_stick_y < -.1) { //Gamepad 1 deadband right stick
            rightY = -gamepad1.right_stick_y;
        } else if (gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1) { //Gamepad 2 deadband left stick
            rightY = (gamepad2.left_stick_y)/2;
        } else {
            rightY = 0;
        }

        //set the power of the motors with the gamepad values
        left.setPower(leftY * 1.0);
        right.setPower(rightY * 1.0);

        //display motor power on the screen
        telemetry.addData("Left Power", leftY);
        telemetry.addData("Right Power", rightY);

        // Vertical Ball Elevator and Sweeper
        if (gamepad1.right_bumper) {
            lift.setPower(.25);             // elevator up
            sweep.setPower(-1);             // sweeper in
        } else {
            if (gamepad1.left_bumper) {
                lift.setPower(-.25);        // elevator down
                sweep.setPower(1);          // sweeper out
            } else {
                lift.setPower(0);           // elevator stop
                // Independent Sweeper control
                if (gamepad1.a) {
                    sweep.setPower(.25);    // sweeper in
                } else if (gamepad1.b) {
                    sweep.setPower(-.25);   // sweeper out
                } else {
                    sweep.setPower(0);      // sweeper stop
                }
            }
        }

        // Ball Shooter
        if (gamepad1.right_trigger > 0) {
            shootspeed(.8);                 // shoot
        } else {
            shootspeed(0);                  // stop
        }

        // Cap ball lift
        if (gamepad2.right_trigger > 0) {
            caplift.setPower(.75);                  // cap ball lift up
        } else if (gamepad2.left_trigger > 0) {
            caplift.setPower(-.75);                 // cap ball lift down
        } else {
            caplift.setPower(0);                    // cap ball lift stop
        }

        // Arms for holding cap ball
        if (gamepad2.a) {
            arm_left.setPosition(120);
            capOpen = true;
        } else if (gamepad2.b) {
            arm_left.setPosition(0);
            capClose = true;
        }
        if (capOpen == true && buttonTime.time() > 1.0) {
            arm_right.setPosition(120);
            capOpen = false;
        } else if (capClose == true && buttonTime.time() > 1.0) {
            arm_right.setPosition(0);
            capClose = false;
        }

        /*
        // Arm for holding cap ball lift arm up
        if (gamepad2.a) {
            arm.setPosition(0);
        } else if (gamepad2.b) {
            arm.setPosition(130);
        }*/
    }

    //----------------------------------
    // Set shooter speed
    //----------------------------------
    private void shootspeed(double speed) {
        shoot_left.setPower(speed);
        shoot_right.setPower(speed);
    }
}