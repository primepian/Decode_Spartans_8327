package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (group = "test")


public class TestColorSensor extends OpMode {
    TestColorSensorMecanism mecanism = new TestColorSensorMecanism();
    TestColorSensorMecanism.DetectedColor detectedColor;
    @Override
    public void init() {
        mecanism.init(hardwareMap);
    }

    @Override
    public void loop() {
        detectedColor = mecanism.getDetectedColor(telemetry);
        telemetry.addData("Color Detected", detectedColor);
    }
}
