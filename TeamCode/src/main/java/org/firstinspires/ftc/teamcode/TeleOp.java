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
    //private DcMotor caplift2;
    private DcMotor shoot_left;
    private DcMotor shoot_right;

    // Servos
    private CRServo sweep;
    private CRServo ball_holder;
    private Servo arm;
    private Servo button_left;
    private Servo button_right;

    // Variavles
    private float leftY = 0;
    private float rightY = 0;
    private boolean capOpen = false;
    private boolean capClose = false;
    private boolean pusherout = false;

    // Loop cycle time stats variables
    private ElapsedTime buttonTime = new ElapsedTime();  // Time into current state
    private ElapsedTime pusherTime = new ElapsedTime();

    @Override
    public void init() {
        //get references to the motors from the hardware map
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        lift = hardwareMap.dcMotor.get("lift");
        caplift = hardwareMap.dcMotor.get("caplift");
        //caplift2 = hardwareMap.dcMotor.get("caplift2");
        shoot_left = hardwareMap.dcMotor.get("shoot_left");
        shoot_right = hardwareMap.dcMotor.get("shoot_right");

        //get references to the servos from the hardware map
        sweep = hardwareMap.crservo.get("sweep");
        ball_holder = hardwareMap.crservo.get("holder");
        arm = hardwareMap.servo.get("arm");
        button_left = hardwareMap.servo.get("button_left");
        button_right = hardwareMap.servo.get("button_right");

        //reverse the right motor
        right.setDirection(DcMotor.Direction.REVERSE);
        shoot_right.setDirection(DcMotor.Direction.REVERSE);

        //set the initial positions for the servos
        arm.setPosition(0);
        button_left.setPosition(0);
        button_right.setPosition(0);

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
            leftY = -(float)(gamepad1.left_stick_y*0.75);
        } else if (gamepad2.right_stick_y > .1 || gamepad2.right_stick_y < -.1) { //Gamepad 2 deadband right stick
            leftY = (float)(gamepad2.right_stick_y*0.4);
        } else {
            leftY = 0;
        }
        if (gamepad1.right_stick_y > .1 || gamepad1.right_stick_y < -.1) { //Gamepad 1 deadband right stick
            rightY = -(float)(gamepad1.right_stick_y*0.75);
        } else if (gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1) { //Gamepad 2 deadband left stick
            rightY = (float)(gamepad2.left_stick_y*0.4);
        } else {
            rightY = 0;
        }

        //set the power of the motors with the gamepad values
        left.setPower(leftY * 1.0);
        right.setPower(rightY * 1.0);

        //display motor power on the screen
        telemetry.addData("Left Power", leftY);
        telemetry.addData("Right Power", rightY);

        //telemetry.addData("CapOpen", capOpen);
        //telemetry.addData("CapClose", capClose);
        //telemetry.addData("Time", buttonTime.time());
        //telemetry.addData("Pusher Time", pusherTime.time());

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
                if (gamepad1.x) {
                    sweep.setPower(-1);     // sweeper in
                } else if (gamepad1.y) {
                    sweep.setPower(1);      // sweeper out
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

        // Button Pusher
        if (gamepad1.a && !pusherout && pusherTime.time() > 0.5) {
            button_left.setPosition(180);
            button_right.setPosition(180);
            pusherout = true;
            pusherTime.reset();
        } else if (gamepad1.a && pusherout && pusherTime.time() > 0.5){
            button_left.setPosition(0);
            button_right.setPosition(0);
            pusherout = false;
            pusherTime.reset();
        }

        // Cap ball lift
        if (gamepad2.right_trigger > 0) {
            caplift.setPower(.75);                  // cap ball lift up
            ball_holder.setPower(-1);
        } else if (gamepad2.left_trigger > 0) {
            caplift.setPower(-.75);                 // cap ball lift down
            ball_holder.setPower(1);
        } else {
            caplift.setPower(0);                    // cap ball lift stop
            ball_holder.setPower(0);
        }

        // Arm for holding cap ball lift arm up
        if (gamepad2.x) {
            arm.setPosition(180);
        } else if (gamepad2.y) {
            arm.setPosition(0);
        }

        // Front ball holding mechanism
        if (gamepad2.a) {
            ball_holder.setPower(.5);
        } else if (gamepad2.b) {
            ball_holder.setPower(-.5);
        } else {
            ball_holder.setPower(0);
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