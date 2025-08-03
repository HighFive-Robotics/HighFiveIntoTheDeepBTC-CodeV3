package org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Intake;

import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.linearSlidesAnalogInputName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.linkageServoLeftName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.linkageServoRightName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.LinearSlides.LinearSlidesAnalogInputVoltages.linearSlideMaxVoltage;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.LinearSlides.LinearSlidesAnalogInputVoltages.linearSlideMinVoltage;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.LinearSlides.maxPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.LinearSlides.minPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.LinearSlides.slidesAuxAutoPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.LinearSlides.slidesAuxPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.LinearSlides.slidesExtendedPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.LinearSlides.slidesRetractedPose;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighServo;

public class LinearSlides implements HighModule {

    HighServo leftLinkageServo, rightLinkageServo;
    States state = States.None;
    private double target;

    public enum States {
        Extended,
        Retracted,
        Aux,
        AuxAuto,
        Custom,
        None
    }

    public LinearSlides(HardwareMap hardwareMap, double initPosition, boolean isAuto) {
        leftLinkageServo = new HighServo(hardwareMap.get(Servo.class , linkageServoLeftName), hardwareMap.get(AnalogInput.class , linearSlidesAnalogInputName), HighServo.RunMode.Standard, initPosition, isAuto);
        rightLinkageServo = new HighServo(hardwareMap.get(Servo.class , linkageServoRightName), HighServo.RunMode.Standard, initPosition, isAuto);
        leftLinkageServo.setAnalogInputCoefficients(0.01 , linearSlideMinVoltage, linearSlideMaxVoltage, minPose, maxPose);
        target = initPosition;
    }

    public void setState(States state) {
        this.state = state;
        switch (state) {
            case Extended:
                setTarget(slidesExtendedPose);
                break;
            case Retracted:
                setTarget(slidesRetractedPose);
                break;
            case Aux:
                setTarget(slidesAuxPose);
                break;
            case AuxAuto:
                setTarget(slidesAuxAutoPose);
                break;
        }
    }

    public void setState(States state, double time) {
        this.state = state;
        switch (state) {
            case Extended:
                setTarget(slidesExtendedPose, time);
                break;
            case Retracted:
                setTarget(slidesRetractedPose, time);
                break;
            case Aux:
                setTarget(slidesAuxPose, time);
                break;
            case AuxAuto:
                setTarget(slidesAuxAutoPose, time);
                break;
        }
    }

    public void setTargetJoyStick(double input) {
        if (input != 0) {
            state = States.Custom;
        }
        target = target + Range.scale(input, -1, 1, -0.008, 0.008);
        target = Range.clip(target, 0, slidesExtendedPose);
    }

    @Override
    public void setTarget(double target) {
        state = States.Custom;
        this.target = Range.clip(target, 0, slidesExtendedPose);
        leftLinkageServo.setPosition(this.target);
        rightLinkageServo.setPosition(this.target);
    }

    @Override
    public void setTarget(double target, double time) {
        state = States.Custom;
        this.target = Range.clip(target, 0, slidesExtendedPose);
        leftLinkageServo.setPosition(this.target, time);
        rightLinkageServo.setPosition(this.target);
    }

    @Override
    public boolean atTarget() {
        return leftLinkageServo.atTarget();
    }

    @Override
    public double getTarget() {
        return target;
    }

    public double getPosition() {
        return leftLinkageServo.getPosition();
    }

    public double getVoltage() {
        return leftLinkageServo.getVoltage();
    }

    public States getState() {
        return state;
    }

    @Override
    public void update() {
        leftLinkageServo.update();
        rightLinkageServo.update();
    }
}
