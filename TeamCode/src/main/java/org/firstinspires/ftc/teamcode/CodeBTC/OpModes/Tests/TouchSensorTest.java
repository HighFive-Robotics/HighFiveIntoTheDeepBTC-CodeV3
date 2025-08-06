package org.firstinspires.ftc.teamcode.CodeBTC.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
public class TouchSensorTest extends LinearOpMode {

    DigitalChannel touchSensor;
    HardwareMap hardwareMap;

    @Override
    public void runOpMode() throws InterruptedException {
        touchSensor = hardwareMap.get(DigitalChannel.class, "TS");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();
        while(opModeIsActive()){
            if(touchSensor.getState() == false)
            {
                telemetry.addData("Sensor is pressed", touchSensor.getState());
            }
            else {
                telemetry.addData("Sensor is not pressed", touchSensor.getState());
            }
            telemetry.update();
        }

    }
}