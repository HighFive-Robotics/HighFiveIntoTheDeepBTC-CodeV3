package org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.wristOuttakeAnalogInputName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.wristOuttakeServoName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.WristOuttake.WristOuttakeAnalogInputVoltage.wristOuttakeMaxVoltage;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.WristOuttake.WristOuttakeAnalogInputVoltage.wristOuttakeMinVoltage;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.WristOuttake.maxPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.WristOuttake.minPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.WristOuttake.waitToComeForTransfer;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.WristOuttake.wristCollectSpecimenPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.WristOuttake.wristSamplePose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.WristOuttake.wristSampleScoreSpecial;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.WristOuttake.wristScoreSpecimenPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.WristOuttake.wristTransferPose;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighServo;

public class WristOuttake implements HighModule {

    HighServo wristServo;
    public States state = States.None;
    private double target;

    public enum States {
        CollectSpecimen,
        ScoreSpecimen,
        Sample,
        Transfer,
        WaitToComeToTransfer,
        ScoreSampleSpecial,
        None
    }

    /**
     *
     * @param hardwareMap
     * @param initPosition
     * @param isAuto
     */
    public WristOuttake(HardwareMap hardwareMap,double initPosition ,boolean isAuto) {
        wristServo = new HighServo(hardwareMap.get(Servo.class, wristOuttakeServoName), hardwareMap.get(AnalogInput.class, wristOuttakeAnalogInputName) ,HighServo.RunMode.Standard,initPosition , isAuto);
        wristServo.setAnalogInputCoefficients(0.025, wristOuttakeMinVoltage, wristOuttakeMaxVoltage, minPose, maxPose);
        target = initPosition;
    }

    /**
     *
     * @param state
     */
    public void setState(States state) {
        this.state = state;
        switch (state) {
            case CollectSpecimen:
                setTarget(wristCollectSpecimenPose);
                break;
            case Sample:
                setTarget(wristSamplePose);
                break;
            case Transfer:
                setTarget(wristTransferPose);
                break;
            case ScoreSpecimen:
                setTarget(wristScoreSpecimenPose);
                break;
            case WaitToComeToTransfer:
                setTarget(waitToComeForTransfer);
                break;
            case ScoreSampleSpecial:
                setTarget(wristSampleScoreSpecial);
                break;
        }
    }

    /**
     *
     * @param state
     * @param time
     */
    public void setState(States state, double time) {
        this.state = state;
        switch (state) {
            case CollectSpecimen:
                setTarget(wristCollectSpecimenPose, time);
                break;
            case Sample:
                setTarget(wristSamplePose, time);
                break;
            case Transfer:
                setTarget(wristTransferPose, time);
                break;
            case ScoreSpecimen:
                setTarget(wristScoreSpecimenPose, time);
                break;
            case WaitToComeToTransfer:
                setTarget(waitToComeForTransfer, time);
                break;
            case ScoreSampleSpecial:
                setTarget(wristSampleScoreSpecial, time);
                break;
        }
    }

    /**
     *
     * @param target
     */
    @Override
    public void setTarget(double target) {
        this.target = target;
        wristServo.setPosition(target);
    }

    /**
     *
     * @param target
     * @param time
     */
    @Override
    public void setTarget(double target, double time) {
        this.target = target;
        wristServo.setPosition(target, time);
    }

    /**
     *
     * @return
     */
    @Override
    public boolean atTarget() {
        return wristServo.atTarget();
    }

    /**
     *
     * @return
     */
    @Override
    public double getTarget() {
        return target;
    }

    /**
     *
     * @return
     */
    public double getPosition() {
        return wristServo.getPosition();
    }

    /**
     *
     * @return
     */
    public double getVoltage() {
        return wristServo.getVoltage();
    }

    /**
     *
     * @return
     */
    public States getState() {
        return state;
    }

    /**
     *
     */
    @Override
    public void update() {
        wristServo.update();
    }
}
