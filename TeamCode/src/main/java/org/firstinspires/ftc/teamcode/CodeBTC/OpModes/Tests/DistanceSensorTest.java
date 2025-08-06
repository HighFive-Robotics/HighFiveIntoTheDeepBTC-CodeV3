package org.firstinspires.ftc.teamcode.CodeBTC.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
public class DistanceSensorTest extends LinearOpMode {
    DistanceSensor distanceSensor;
    HardwareMap hardwareMap;
    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensor = hardwareMap.get(DistanceSensor.class,"DS");
        waitForStart();
        while (opModeIsActive()) {
            if(distanceSensor.getDistance(DistanceUnit.CM)<=200)
            {
                telemetry.addData("close", distanceSensor.getDistance(DistanceUnit.CM));
            }
            else {
                telemetry.addData("far", distanceSensor.getDistance(DistanceUnit.CM));
            }
            telemetry.update();
        }

    }
}