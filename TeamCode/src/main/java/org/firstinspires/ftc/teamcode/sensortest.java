package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import java.lang.Override;

/**
 * @author Ryan Kirkpatrick
 * @version 11/16/2016
 */
@Autonomous(name = "sensorTest", group = "Test Programs")
public class sensortest extends OpMode {

    private byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

    private I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    private static final int RANGE1_REG_START = 0x04; //Register to start reading
    private static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    private I2cDevice RANGE1;
    private I2cDeviceSynch RANGE1Reader;

    @Override
    public void init() {
        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
    }

    @Override
    public void loop() {
        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

        telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
        telemetry.addData("ODS", range1Cache[1] & 0xFF);
    }

    @Override
    public void stop() {

    }
}
