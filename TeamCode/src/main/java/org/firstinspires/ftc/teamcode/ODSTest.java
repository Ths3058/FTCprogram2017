package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "ODSTest", group = "Autonomous")
public class ODSTest extends OpMode {

    OpticalDistanceSensor opticalDistanceSensorleft;
    OpticalDistanceSensor opticalDistanceSensorright;

    @Override
    public void init() {
        opticalDistanceSensorleft = hardwareMap.opticalDistanceSensor.get("ODS_left");
        opticalDistanceSensorright = hardwareMap.opticalDistanceSensor.get("ODS_right");
    }

    @Override
    public void loop() {
        telemetry.addData("Left: ", opticalDistanceSensorleft.getLightDetected()); // white = 0.3
        telemetry.addData("Right: ", opticalDistanceSensorright.getLightDetected());
    }
}