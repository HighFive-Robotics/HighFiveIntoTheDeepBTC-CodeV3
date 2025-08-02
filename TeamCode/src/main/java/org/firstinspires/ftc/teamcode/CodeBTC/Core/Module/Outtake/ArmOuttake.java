package org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.ArmOuttakeAnalogInputVoltage.armOuttakeMaxVoltage;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.ArmOuttakeAnalogInputVoltage.armOuttakeMinVoltage;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.ArmOuttakePoses.armCollectSpecimenPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.ArmOuttakePoses.armCollectSpecimenSpecialPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.ArmOuttakePoses.armCollectSpecimenSpecialSpecialPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.ArmOuttakePoses.armScoreSamplePose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.ArmOuttakePoses.armScoreSpecimenPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.ArmOuttakePoses.armTransferPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.ArmOuttakePoses.maxPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.ArmOuttakePoses.minPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.ArmOuttakePoses.waitScorePose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.armOuttakeAnalogInputName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.armOuttakeLeftServoName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.armOuttakeRightServoName;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighServo;

public class ArmOuttake implements HighModule {

    public HighServo armServoLeft , armServoRight;
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

    public ArmOuttake(HardwareMap hardwareMap, double initPosition ,boolean isAutonomous) {
        armServoLeft = new HighServo(hardwareMap.get(Servo.class , armOuttakeLeftServoName), hardwareMap.get(AnalogInput.class , armOuttakeAnalogInputName) ,HighServo.RunMode.MotionProfiler , initPosition , isAutonomous);
        armServoRight = new HighServo(hardwareMap.get(Servo.class , armOuttakeRightServoName),HighServo.RunMode.MotionProfiler , initPosition, isAutonomous);
        setMotionCoefficients(motionProfileCoefficientsGoingUp);
        armServoLeft.setAnalogInputCoefficients(0.04 ,armOuttakeMinVoltage, armOuttakeMaxVoltage, minPose, maxPose);
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
        armServoLeft.setPosition(target);
        armServoRight.setPosition(target);
    }

    @Override
    public void setTarget(double target, double time) {
        this.target = target;
        armServoLeft.setPosition(target, time);
        armServoRight.setPosition(target);
    }

    @Override
    public boolean atTarget() {
        return armServoLeft.atTarget();
    }

    @Override
    public double getTarget() {
        return target;
    }

    public double getPosition() {
        return armServoLeft.getPosition();
    }

    public double getVoltage() {
        return armServoLeft.getVoltage();
    }

    public States getState() {
        return state;
    }

    @Override
    public void update() {
        armServoRight.update();
        armServoLeft.update();
    }

    public void setMotionCoefficients(double [] vector) {
        armServoLeft.setMotionProfilerCoefficients(vector[0], vector[1], vector[2]);
        armServoRight.setMotionProfilerCoefficients(vector[0], vector[1], vector[2]);
    }
}
