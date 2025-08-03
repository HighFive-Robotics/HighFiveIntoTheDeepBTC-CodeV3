package org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Outtake;


import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.clawServoName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.Claw.clawClosePose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.Claw.clawOpenPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.Claw.clawSemiClosePose;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighServo;

public class Claw implements HighModule {

    public HighServo claw;
    public States state = States.None;
    private double target;

    public enum States{
        Closed,
        ClosedTransfer,
        Open,
        None
    }

    /**
     *
     * @param hardwareMap
     * @param initPosition
     * @param isAuto
     */
    public Claw(HardwareMap hardwareMap, double initPosition, boolean isAuto) {
        claw = new HighServo(hardwareMap.get(Servo.class, clawServoName), HighServo.RunMode.Standard, initPosition, isAuto);
        target = clawClosePose;
    }

    /**
     *
     * @param state
     */
    public void setState(States state){
        this.state = state;
        switch(state){
            case Closed:
                setTarget(clawClosePose);
                break;
            case ClosedTransfer:
                setTarget(clawSemiClosePose);
                break;
            case Open:
                setTarget(clawOpenPose);
                break;
        }
    }

    /**
     *
     * @param state
     * @param time
     */
    public void setState(States state, double time){
        this.state = state;
        switch(state){
            case Closed:
                setTarget(clawClosePose, time);
                break;
            case ClosedTransfer:
                setTarget(clawSemiClosePose, time);
                break;
            case Open:
                setTarget(clawOpenPose, time);
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
        claw.setPosition(target);
    }

    /**
     *
     * @param target
     * @param time
     */
    @Override
    public void setTarget(double target, double time) {
        this.target = target;
        claw.setPosition(target, time);
    }

    /**
     *
     * @return
     */
    @Override
    public boolean atTarget() {
        return claw.atTarget();
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
    public States getState() {
        return state;
    }

    /**
     *
     */
    @Override
    public void update() {
        claw.update();
    }
}
