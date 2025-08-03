package org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.linkageOuttakeAnalogInputName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.linkageOuttakeServoName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.LinkageOuttake.LinkageOuttakeVoltages.slidesOuttakeMaxVoltage;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.LinkageOuttake.LinkageOuttakeVoltages.slidesOuttakeMinVoltage;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.LinkageOuttake.maxPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.LinkageOuttake.minPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.LinkageOuttake.slidesOuttakeAuxPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.LinkageOuttake.slidesOuttakeExtendedPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.LinkageOuttake.slidesOuttakeRetractedPose;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighServo;

public class LinkageOuttake implements HighModule {

    public HighServo linkageServo;
    States state = States.None;
    private double target;

    public enum States {
        Extended,
        Retracted,
        Aux,
        None
    }


    public LinkageOuttake(HardwareMap hardwareMap, double initPosition , boolean isAuto) {
        linkageServo = new HighServo(hardwareMap.get(Servo.class, linkageOuttakeServoName), hardwareMap.get(AnalogInput.class, linkageOuttakeAnalogInputName) ,HighServo.RunMode.Standard,initPosition , isAuto);
        linkageServo.setAnalogInputCoefficients(0.025, slidesOuttakeMinVoltage, slidesOuttakeMaxVoltage, minPose, maxPose);
        target = initPosition;
    }

    public void setState(States state) {
        this.state = state;
        switch (state) {
            case Extended:
                setTarget(slidesOuttakeExtendedPose);
                break;
            case Retracted:
                setTarget(slidesOuttakeRetractedPose);
                break;
            case Aux:
                setTarget(slidesOuttakeAuxPose);
                break;
        }
    }

    public void setState(States state, double time) {
        this.state = state;
        switch (state) {
            case Extended:
                setTarget(slidesOuttakeExtendedPose, time);
                break;
            case Retracted:
                setTarget(slidesOuttakeRetractedPose, time);
                break;
            case Aux:
                setTarget(slidesOuttakeAuxPose, time);
                break;
        }
    }

    @Override
    public void setTarget(double target) {
        this.target = target;
        linkageServo.setPosition(target);
    }

    @Override
    public void setTarget(double target, double time) {
        this.target = target;
        linkageServo.setPosition(target, time);
    }

    @Override
    public boolean atTarget() {
        return linkageServo.atTarget();
    }

    @Override
    public double getTarget() {
        return target;
    }

    public double getPosition() {
        return linkageServo.getPosition();
    }

    public double getVoltage() {
        return linkageServo.getVoltage();
    }

    public States getState() {
        return state;
    }

    @Override
    public void update() {
        linkageServo.update();
    }
}
