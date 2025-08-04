package org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Intake;

import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.wristIntakeAnalogInputName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.wristIntakeServoName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.wristOuttakeAnalogInputName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.wristOuttakeServoName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.WristIntake.WristIntakeAnalogInputVoltage.wristIntakeMaxVoltage;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.WristIntake.WristIntakeAnalogInputVoltage.wristIntakeMinVoltage;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.WristIntake.maxPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.WristIntake.minPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.WristIntake.wristCollectPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.WristIntake.wristCollectSpecificOnePose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.WristIntake.wristPushBadColor;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.WristIntake.wristSpecimenWait;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.WristIntake.wristSpitPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.WristIntake.wristTransferPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.WristIntake.wristWaitCollectSpecificOnePose;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighServo;

public class WristIntake implements HighModule {

    HighServo wristServo;
    public States state = States.None;
    private double target;

    public enum States{
        CollectSpecificOne,
        WaitCollectSpecificOne,
        Collect,
        SpecimenWait,
        Spit,
        Transfer,
        PushBadColor,
        None
    }

    /**
     *
     * @param hardwareMap
     * @param initPosition
     * @param isAuto
     */
    public WristIntake(HardwareMap hardwareMap,double initPosition ,boolean isAuto) {
        wristServo = new HighServo(hardwareMap.get(Servo.class, wristIntakeServoName), hardwareMap.get(AnalogInput.class, wristIntakeAnalogInputName) ,HighServo.RunMode.Standard,initPosition , isAuto);
        wristServo.setAnalogInputCoefficients(0.025, wristIntakeMinVoltage, wristIntakeMaxVoltage, minPose, maxPose);
        target = initPosition;
    }

    public void setState(States state) {
        this.state=state;
        switch (state){
            case CollectSpecificOne:
                setTarget(wristCollectSpecificOnePose);
                break;
            case WaitCollectSpecificOne:
                setTarget(wristWaitCollectSpecificOnePose);
                break;
            case Collect:
                setTarget(wristCollectPose);
                break;
            case SpecimenWait:
                setTarget(wristSpecimenWait);
                break;
            case Spit:
                setTarget(wristSpitPose);
                break;
            case Transfer:
                setTarget(wristTransferPose);
                break;
            case PushBadColor:
                setTarget(wristPushBadColor);
                break;
        }
    }

    public void setState(States state, double time) {
        this.state=state;
        switch (state){
            case CollectSpecificOne:
                setTarget(wristCollectSpecificOnePose, time);
                break;
            case WaitCollectSpecificOne:
                setTarget(wristWaitCollectSpecificOnePose, time);
                break;
            case Collect:
                setTarget(wristCollectPose, time);
                break;
            case SpecimenWait:
                setTarget(wristSpecimenWait, time);
                break;
            case Spit:
                setTarget(wristSpitPose, time);
                break;
            case Transfer:
                setTarget(wristTransferPose, time);
                break;
            case PushBadColor:
                setTarget(wristPushBadColor, time);
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
