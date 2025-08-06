package org.firstinspires.ftc.teamcode.CodeBTC.OpModes.Tests;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
public class ColorSensorTest extends LinearOpMode {

    ColorSensor colorSensor;
    HardwareMap hardwareMap;

    @Override
    public void runOpMode() throws InterruptedException {

        colorSensor=hardwareMap.get(ColorSensor.class,"CS");
        float[] hsvValues= new float[3];

        waitForStart();

        while (opModeIsActive()) {

            int red=colorSensor.red();
            int green=colorSensor.green();
            int blue=colorSensor.blue();

            Color.RGBToHSV(red*8,green*8, blue*8, hsvValues);

            float hue=hsvValues[0];

            String detectedColor;

            if( (hue>=0 && hue<30) || (hue>=330 && hue<=360))
                detectedColor="Red";
            else if(hue>=40 && hue<=65)
            {
                detectedColor="Yellow";
            }else if(hue>=200 && hue<=250) {
                detectedColor="Blue";
            }else detectedColor="Other";

            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Hue", hue);

            telemetry.addData("Detected Color", detectedColor);
            telemetry.update();

        }

    }
}