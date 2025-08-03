package org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Outtake;


import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.armOuttakeAnalogInputName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.armOuttakeLeftServoName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.armOuttakeRightServoName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.ArmOuttake.ArmOuttakeAnalogInputVoltage.armOuttakeMaxVoltage;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.ArmOuttake.ArmOuttakeAnalogInputVoltage.armOuttakeMinVoltage;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.ArmOuttake.armCollectSpecimenPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.ArmOuttake.armCollectSpecimenSpecialPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.ArmOuttake.armCollectSpecimenSpecialSpecialPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.ArmOuttake.armScoreSamplePose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.ArmOuttake.armScoreSpecimenPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.ArmOuttake.armTransferPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.ArmOuttake.maxPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.ArmOuttake.minPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.ArmOuttake.waitScorePose;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighServo;

public class ArmOuttake implements HighModule {

    public HighServo leftArmOuttake, rightArmOuttake;
    public States state = States.None;
    private final double [] motionProfileCoefficientsGoingDown = {6, 6, 2};
    private final double [] motionProfileCoefficientsGoingUp = {5, 5, 3};
    private double target;

    public enum States {
        CollectSpecimen,
        CollectSpecialSpecimen,
        CollectSpecialSpecialSpecimen,
        Transfer,
        ScoreSpecimen,
        ScoreSample,
        WaitScoreSample,
        None
    }

    public ArmOuttake(HardwareMap hardwareMap, double initPosition , boolean isAuto) {
        leftArmOuttake = new HighServo(hardwareMap.get(Servo.class , armOuttakeLeftServoName), hardwareMap.get(AnalogInput.class , armOuttakeAnalogInputName) ,HighServo.RunMode.MotionProfiler , initPosition , isAuto);
        rightArmOuttake = new HighServo(hardwareMap.get(Servo.class , armOuttakeRightServoName),HighServo.RunMode.MotionProfiler , initPosition, isAuto);
        setMotionCoefficients(motionProfileCoefficientsGoingUp);
        leftArmOuttake.setAnalogInputCoefficients(0.04 ,armOuttakeMinVoltage, armOuttakeMaxVoltage, minPose, maxPose);
        target = initPosition;
    }

    public void setState(States state) {
        this.state = state;
        switch (state) {
            case CollectSpecimen:
                setTarget(armCollectSpecimenPose);
                setMotionCoefficients(motionProfileCoefficientsGoingDown);
                break;
            case CollectSpecialSpecimen:
                setTarget(armCollectSpecimenSpecialPose);
                setMotionCoefficients(motionProfileCoefficientsGoingDown);
                break;
            case CollectSpecialSpecialSpecimen:
                setTarget(armCollectSpecimenSpecialSpecialPose);
                setMotionCoefficients(motionProfileCoefficientsGoingDown);
                break;
            case Transfer:
                setTarget(armTransferPose);
                setMotionCoefficients(motionProfileCoefficientsGoingDown);
                break;
            case ScoreSpecimen:
                setTarget(armScoreSpecimenPose);
                setMotionCoefficients(motionProfileCoefficientsGoingUp);
                break;
            case ScoreSample:
                setTarget(armScoreSamplePose);
                setMotionCoefficients(motionProfileCoefficientsGoingUp);
                break;
            case WaitScoreSample:
                setTarget(waitScorePose);
                setMotionCoefficients(motionProfileCoefficientsGoingUp);
                break;

        }
    }

    public void setState(States state, double time) {
        this.state = state;
        switch (state) {
            case CollectSpecimen:
                setTarget(armCollectSpecimenPose, time);
                setMotionCoefficients(motionProfileCoefficientsGoingDown);
                break;
            case CollectSpecialSpecimen:
                setTarget(armCollectSpecimenSpecialPose, time);
                setMotionCoefficients(motionProfileCoefficientsGoingDown);
                break;
            case CollectSpecialSpecialSpecimen:
                setTarget(armCollectSpecimenSpecialSpecialPose, time);
                setMotionCoefficients(motionProfileCoefficientsGoingDown);
                break;
            case Transfer:
                setTarget(armTransferPose, time);
                setMotionCoefficients(motionProfileCoefficientsGoingDown);
                break;
            case ScoreSpecimen:
                setTarget(armScoreSpecimenPose, time);
                setMotionCoefficients(motionProfileCoefficientsGoingUp);
                break;
            case ScoreSample:
                setTarget(armScoreSamplePose, time);
                setMotionCoefficients(motionProfileCoefficientsGoingUp);
                break;
            case WaitScoreSample:
                setTarget(waitScorePose, time);
                setMotionCoefficients(motionProfileCoefficientsGoingUp);
                break;

        }
    }

    @Override
    public void setTarget(double target) {
        this.target = target;
        leftArmOuttake.setPosition(target);
        rightArmOuttake.setPosition(target);
    }

    @Override
    public void setTarget(double target, double time) {
        this.target = target;
        leftArmOuttake.setPosition(target, time);
        rightArmOuttake.setPosition(target);
    }

    @Override
    public boolean atTarget() {
        return leftArmOuttake.atTarget();
    }

    @Override
    public double getTarget() {
        return target;
    }

    public double getPosition() {
        return leftArmOuttake.getPosition();
    }

    public double getVoltage() {
        return leftArmOuttake.getVoltage();
    }

    public States getState() {
        return state;
    }

    @Override
    public void update() {
        rightArmOuttake.update();
        leftArmOuttake.update();
    }

    public void setMotionCoefficients(double [] vector) {
        leftArmOuttake.setMotionProfilerCoefficients(vector[0], vector[1], vector[2]);
        rightArmOuttake.setMotionProfilerCoefficients(vector[0], vector[1], vector[2]);
    }
}
