package org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Others;

import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Color.None;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.led1Name;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.led2Name;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CodeBTC.Constants;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighModuleSimple;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighServo;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Intake.ActiveIntake;


public class Led implements HighModuleSimple {

    Servo led1, led2;

    public Led(HardwareMap hw) {
        led1 = hw.get(Servo.class, led1Name);
        led2 = hw.get(Servo.class, led2Name);
    }

    Constants.Color lastColor = None;

    @Override
    public void update() {
        if(ActiveIntake.color != lastColor){
            if (ActiveIntake.color == Constants.Color.Blue) {
                led1.setPosition(0.611);
                led2.setPosition(0.611);//0.5 color for green
            } else if (ActiveIntake.color == Constants.Color.Yellow) {
                led1.setPosition(0.345);
                led2.setPosition(0.345);
            } else if (ActiveIntake.color == Constants.Color.Red) {
                led1.setPosition(0.279);
                led2.setPosition(0.279);
            } else {
                led1.setPosition(0);
                led2.setPosition(0);
            }
            lastColor = ActiveIntake.color;
        }
    }
}