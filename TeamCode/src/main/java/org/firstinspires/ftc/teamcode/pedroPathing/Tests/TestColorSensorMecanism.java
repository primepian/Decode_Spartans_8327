package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestColorSensorMecanism {
    Servo barril;
    NormalizedColorSensor colorSensor;

    public enum DetectedColor{
        PURPLE,
        GREEN,
        UNKNOWN,
    }

    public void init(HardwareMap hwMap){
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");
        barril = hwMap.get(Servo.class,"servo");
        colorSensor.setGain(10);
    }

    public DetectedColor getDetectedColor(Telemetry telemetry){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);

        //TODO colors
        /*
         GREEN = <.5, >.3, <.5
         PURPLE =  <.25, <.25, >.25
         */

        if (normRed < 0.075 && normGreen > 0.11 && normBlue < 0.20) {
            return DetectedColor.GREEN;
        } else if (normRed < 0.25 && normGreen < 0.25 && normBlue > 0.16) {
            return DetectedColor.PURPLE;
        } else {
            return DetectedColor.UNKNOWN;
        }
    }
}
