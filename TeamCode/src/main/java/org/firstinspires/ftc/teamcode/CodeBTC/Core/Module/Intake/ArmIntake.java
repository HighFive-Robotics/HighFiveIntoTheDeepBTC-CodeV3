package org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Intake;




import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.armIntakeAnalogInputName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.armIntakeServoName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.ArmIntake.ArmIntakeAnalogInputVoltage.armIntakeMaxVoltage;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.ArmIntake.ArmIntakeAnalogInputVoltage.armIntakeMinVoltage;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.ArmIntake.armSpecimenWaitPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.ArmIntake.armSpitPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.ArmIntake.armWaitSpecificPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.ArmIntake.armWaitPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.ArmIntake.collectPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.ArmIntake.collectSpecificOnePose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.ArmIntake.maxPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.ArmIntake.minPose;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.ArmIntake.transferPose;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighModule;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighServo;

public class ArmIntake implements HighModule {

    public HighServo armServo;
    public States state = States.None;
    private double target;

    public enum States {
        Collect,
        CollectSpecificOne,
        WaitSpecificOne,
        WaitCollecting,
        SpecimenWait,
        Transfer,
        Spit,
        None
    }

    public ArmIntake(HardwareMap hardwareMap, double initPosition , boolean isAuto) {
        armServo = new HighServo(hardwareMap.get(Servo.class , armIntakeServoName), hardwareMap.get(AnalogInput.class , armIntakeAnalogInputName) ,HighServo.RunMode.Standard , initPosition , isAuto);
        armServo.setAnalogInputCoefficients(0.1 ,armIntakeMinVoltage, armIntakeMaxVoltage, minPose, maxPose);
        target = initPosition;
    }

    public void setState(ArmIntake.States state) {
        this.state=state;
        switch (state){
            case Collect:
                setTarget(collectPose);
                break;
            case CollectSpecificOne:
                setTarget(collectSpecificOnePose);
                break;
            case WaitSpecificOne:
                setTarget(armWaitSpecificPose);
                break;
            case WaitCollecting:
                setTarget(armWaitPose);
                break;
            case SpecimenWait:
                setTarget(armSpecimenWaitPose);
                break;
            case Spit:
                setTarget(armSpitPose);
                break;
            case Transfer:
                setTarget(transferPose);
                break;
        }
    }

    public void setState(ArmIntake.States state, double time) {
        this.state=state;
        switch (state){
            case Collect:
                setTarget(collectPose, time);
                break;
            case CollectSpecificOne:
                setTarget(collectSpecificOnePose, time);
                break;
            case WaitSpecificOne:
                setTarget(armWaitSpecificPose, time);
                break;
            case WaitCollecting:
                setTarget(armWaitPose, time);
                break;
            case SpecimenWait:
                setTarget(armSpecimenWaitPose, time);
                break;
            case Spit:
                setTarget(armSpitPose, time);
                break;
            case Transfer:
                setTarget(transferPose, time);
                break;
        }
    }

    @Override
    public void setTarget(double target) {
        this.target = target;
        armServo.setPosition(target);
    }

    @Override
    public void setTarget(double target, double time) {
        this.target = target;
        armServo.setPosition(target, time);
    }

    @Override
    public boolean atTarget() {
        return armServo.atTarget();
    }

    @Override
    public double getTarget() {
        return target;
    }

    public double getPosition() {
        return armServo.getPosition();
    }

    public double getVoltage() {
        return armServo.getVoltage();
    }

    public States getState() {
        return state;
    }

    @Override
    public void update() {
        armServo.update();
    }

}
