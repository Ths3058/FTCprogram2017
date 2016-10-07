package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.lang.Override;

/**
 * @author Ryan Kirkpatrick
 * @version 9/22/16
 */
@Autonomous(name = "test", group = "Test Programs")
@Disabled
public class test extends OpMode {
    //Motors
    DcMotor left;
    DcMotor right;

    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");

        left.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        left.setPower(1);
        right.setPower(1);
    }
}